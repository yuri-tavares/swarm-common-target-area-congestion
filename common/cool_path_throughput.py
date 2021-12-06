from math import pi,asin,floor,sin,cos

def delay(s,d,K):
  angle = 2*pi/K
  rd = ((s/d)*sin(angle/2) - 0.5)/(1-sin(angle/2))
  if (2 * rd * cos(angle / 2) < 1):
    de = (rd * (pi - angle))  + (1 - 2 * rd * cos(angle / 2)) / (sin(angle / 2))
  else:
    de = 2 * rd * asin(0.5/rd)
  de = max(de, 1)
  return de

def Kmax(s,d):
  km = 3
  maxi = 0
  K = int(floor(pi/asin(0.5/(s/d))))
  for i in range(3,K+1):
    de = delay(s,d,i)
    va = i/de
    if (maxi < va):
      maxi = va
      km = i
  return km

def limitF(K,s,v,d):
  angle = 2*pi/K
  r = (s*sin(angle/2) - d/2.)/(1-sin(angle/2))
  if (2 * r * cos(angle / 2) < d): 
    delay = (r * (pi - angle))  + (d - 2 * r * cos(angle / 2)) / (sin(angle / 2));
  else:
    delay = 2 * r * asin(d / (2 * r));
  delay = max(delay, d);
  return K*v/delay
