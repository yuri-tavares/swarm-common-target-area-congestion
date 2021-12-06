import numpy as np
from math import tan, sin, cos, pi, sqrt, floor, ceil, hypot, log, fabs, fmod

waypoint = (100,100)
epsilon = 0.001

def intfloor(x):
  return int(floor(x))

def intceil(x):
  return int(ceil(x))

def Rotation(x,y,lx,ly,th):
  M = np.array([[cos(th), -sin(th)],[sin(th),cos(th)]])
  return M.dot((x-lx,y-ly))

def countSemicircularTail(th, v, T, s, d, x0, y0, lastX, lastY):
  '''
    Counts the number of robot inside the semicircular tail of the corridor
    Arguments:
      th: angle of the hexagonal tiling in radians;
      v: linear speed;
      T: time;
      s,d: target region radius and distance from each robot;
      x0,y0: Position of the first robot to reach the target;
      lastX, lastY: values of the last robot coordinates in the rectangular part
  '''
  cx = x0 + v*T - s;
  if T > s/v:
    B = intceil((2*(sin(pi/3-th)*(cx-lastX) + cos(pi/3-th)*(y0-lastY-s)))/(sqrt(3)*d));
  else:
    B = intceil(-((2*sqrt(2*s*v*T-(v*T)**2))/(sqrt(3)*d))*sin(th+pi/6)) 
  if T > s/v or atan((s/2 - sin(th)*(v*T-s))/(sqrt(3)*s/2 + cos(th)*(v*T - s))) < pi/2 - th:
    U = intfloor((2*(sin(pi/3-th)*(cx-lastX)+cos(pi/3-th)*(y0-lastY)+s))/(sqrt(3)*d))
  else:
    U = intfloor(((2*sqrt(2*s*v*T - (v*T)**2))/(sqrt(3)*d))*cos(th-pi/3))
  cx,cy = Rotation(cx,y0,lastX,lastY,-th);
  S = 0;
  for xh in range(B,U+1):
    Delta = 4*s**2 - (sqrt(3)*(d*xh-cx) - cy)**2
    if (Delta < 0): continue;
    part1 = d*xh - cx + sqrt(3)*cy
    Y1S = (part1 - sqrt(Delta))/(2*d);
    C2  = (part1 + sqrt(Delta))/(2*d);
    if T > s/v:
      L = (sin(pi/2 - th)*(d*xh - cx) + cos(pi/2-th)*cy)/(d*sin(5*pi/6 - th))
    else:
      L = sin(pi/2 - th)*xh/sin(5*pi/6 - th)
    Y2S = min(L,C2)
    if fabs(Y2S - floor(L)) <= epsilon:
      Y2S -= 1
    V = intfloor(Y2S)- intceil(Y1S)+1;
    if (V < 0): continue;
    S += V;
  return S;

def countHexagonalTiling(T,s,d,v,th,lastX,lastY,appendTail = True):
  nm = intfloor((2*s*sin(abs(pi/6 - th)))/(sqrt(3)*d))
  np = intfloor((2*(v*T-s)*cos(pi/6-th) + 2*s*sin(abs(pi/6 - th)))/(sqrt(3)*d))
  S = 0
  for xh in range(-nm,np+1):
    ROUND = 13
    if th == pi/6:
      Y1R = round(xh/2-s/d,ROUND)
      Y2R = round(xh/2+s/d,ROUND)
    else:
      vA = round((sin(pi/3-th)*xh-s/d)/cos(th-pi/6), ROUND);
      vB = round(-cos(pi/3-th)*xh/sin(pi/6-th), ROUND);
      vC = round(((v*T-s)/d - cos(pi/3-th)*xh)/sin(pi/6-th), ROUND); 
      vD = round((sin(pi/3-th)*xh+s/d)/cos(th-pi/6), ROUND);
      if th < pi/6:
        Y1R = max(vA,vB)
        Y2R = min(vD,vC)
      else:
        Y1R = max(vA,vC)
        Y2R = min(vD,vB)
    V = intfloor(Y2R) - intceil(Y1R) + 1
    if V < 0: continue
    S += V
  if appendTail:
    x0=waypoint[0]+s
    y0=waypoint[1]
    Ssc = countSemicircularTail(th, v, T, s, d, x0, y0, lastX, lastY)
    rS = S + Ssc
  else:
    rS = S
  return rS


def discount(i, th, d, x0, px):
  '''
  Returns the distance from vertical line in x0 to starting robot at queue i, inclined by angle th, in a queue starting at px in x-axis.  
  '''
  if i >= 0:
    R = fabs(fmod(((x0 + i*d*cos(th+2*pi/3) - px)/cos(th)) , d))
  else:
    R = fabs(fmod(((x0 - i*d*cos(th-2*pi/3) - px)/cos(th)) , d))
  return R


def updateIfItIsTheLastRobot(lastX, lastY, px, py, x0, y0, s, v, T):
  '''
  Updates the variables lastX and lastY if the robot is nearer to the corridor's semicircular tail centre than the previous value lastX and lastY. These variables need to be initialised with (x0,y0) before the first use.
  Arguments:
    lastX, lastY: previous values of the possible last robot coordinates (passed by reference);
    px, py: new values to evaluate;
    x0,y0: coordinates of the first robot to reach the target;
    s: target area's radius;
    v: linear velocity;
    T: time from first robot arrival at target region.
  '''
  cy = y0; 
  if fabs(v*T - s + x0 - px) + fabs(y0 - py) < fabs(v*T - s + x0 - lastX) + fabs(y0 - lastY):
    lastX = px;
    lastY = py;
  return lastX,lastY

def rangePointsInRectangle(P, d, th, x0, y0, Y1, Y2, lastX, lastY, v, T):
  '''
  Returns a list of two-dimensional points starting at P, spaced by d units, in the direction of the angle th while the x coordinate is in [x0, x0 + X] and the y coordinate is in [y0 - Y1, y0 + Y2], i.e., the point is inside the corridor for X = vT - s. Also, it outputs the coordinate of the last robot (lastX, lastY).
  Arguments:
    P: starting point;
    d: distance between points;
    th: orientation angle in radians;
    (x0,y0): base point;
    Y1, Y2: top and bottom widths measured from y0. Y1 + Y2 is the height of the corridor;
    lastX, lastY: previous values of the possible last robot coordinates;
    v: linear velocity;
    T: time from first robot arrival at target region.
  Return:
    L: the list
    lastX, lastY: last robot coordinates until the moment.
  '''
  s = (Y1+Y2)/2.
  X = v*T - s
  L = []
  p = P;
  dcth = d*cos(th); dsth = d*sin(th);
  while (x0 - epsilon/2 <= p[0] and p[0] <= x0 + X + epsilon/2 and y0 - Y1 -epsilon/2 <= p[1] and p[1] <= y0 + Y2 + epsilon/2):
    lastX, lastY = updateIfItIsTheLastRobot(lastX, lastY, p[0], p[1], x0, y0, s, v, T);
    L.append(p);
    p = (p[0] + dcth, p[1] + dsth)
  return L,lastX,lastY;

def findLastRobot(th, Y1, Y2, d, x0, y0, v, T):
  '''
  Returns a list of robots' position and orientation inside a corridor using hexagonal tiling. The corridor measures X x (Y1 + Y2) units of area. Also, it returns the pair (lastX,lastY) inside the rectangular corridor.
    
    |------- X ------|
    |- X - s --|- s -|
    _________________
                     ⎞
    (cx,cy) -> o     ⎟
    _________________⎠
    
    Arguments:
      th: angle of the hexagonal tiling in radians;
      (x0,y0): Position of the first robot to reach the target;
      Y1, Y2: top and bottom widths measured from y0. Y1 + Y2 is the height of the corridor;
      d: distance from each robot;
      v: linear velocity;
      T: time from first robot arrival at target region. vT = X
    Return:
      lastX, lastY: values of the last robot coordinates.
  '''
  X = v*T
  s = (Y1 + Y2)/2.;
  X2 = X - s;
  L = [];
  h1 = intfloor((Y2*cos(th))/(sqrt(3)*d/2)); #U from paper
  h2 = intfloor((X2*sin(th) + Y1*cos(th))/(sqrt(3)*d/2)); #-T' from paper
  P = (x0, y0)
  lastX = x0;
  lastY = y0;
  if X2 < 0: 
    return lastX,lastY
  L0,lastX,lastY = rangePointsInRectangle(P,d,th,x0,y0,Y1,Y2,lastX,lastY,v,T);
  if (len(L0)>0): L.append(L0);
  for i in range(1,h1+1):
    di = discount(i,th,d,x0,x0);
    ''' I need to find out why, when I use 30 degrees, the discount does not need to be the complement in Z_d (i.e., d - discount). Maybe it is because, although mod(d - mod(q,d),d) = mod(-q,d), numerically it is not!''' 
    if (th != pi/6 and th != 0): di = d - di;
    dx = di*cos(th);
    dy = di*sin(th);
    P = (x0+dx, y0 + i * (sqrt(3)*d/(2*cos(th)))+dy)
    L1,lastX,lastY = rangePointsInRectangle(P,d,th,x0,y0,Y1,Y2,lastX,lastY,v,T);
    if (len(L1)>0): L.append(L1);
  for i in range(-h2,0,-1):
    if (i == -1):
      if L0 != []:
        P = ( L0[0][0] + d*cos(th-2.*pi/3.), L0[0][1] + d*sin(th-2.*pi/3.))
      else:
        break
    else:
      P = (L[len(L)-1][0][0] + d*cos(th-2.*pi/3.), L[len(L)-1][0][1] + d*sin(th-2.*pi/3.))
    # I is how many ds I need to add or subtract to P
    if (y0 - Y1 <= P[1]):
      I = intfloor(((-x0 + P[0]   + epsilon/2)/cos(th))/d);
      P = (P[0] - I*d*cos(th), P[1] - I*d*sin(th))
    if (y0 - Y1 > P[1] and fmod(th,pi) != 0. ): # this is not a 'else', because after the previous 'if' the y coord. of P could be greater than y0 - Y1
      I = intceil((((y0 - Y1) - P[1] - epsilon/2)/sin(th))/d);
      P = (P[0] + (I)*d*cos(th), P[1] + (I)*d*sin(th));
    L1,lastX,lastY = rangePointsInRectangle(P,d,th,x0,y0,Y1,Y2,lastX,lastY,v,T);
    if (len(L1)>0): L.append(L1);
  return lastX,lastY

def throughput(T,s,d,v,th):
  lastX,lastY = findLastRobot(th, s, s, d, waypoint[0]+s, waypoint[1], v, T)
  N = countHexagonalTiling(T,s,d,v,th,lastX,lastY)
  return (N-1)/T


def bestThroughput(T,s,d,v,numPoints):
  ths = np.linspace(0,pi/3, num=numPoints)
  f = [throughput(T,s,d,v,theta) for theta in ths]
  maxf = max(f)
  return (maxf,ths[f.index(maxf)])

def bestTheta(T,s,d,v,numPoints):
  (m1,t1) = bestThroughput(T,s,d,v,numPoints)
  (m2,t2) = (throughput(T,s,d,v,pi/6),pi/6)
  if m1 > m2: return (m1,t1)
  else: return (m2,t2)
