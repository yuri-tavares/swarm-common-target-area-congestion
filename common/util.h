#ifndef _UTIL_H_
#define _UTIL_H_

#include <string>
#include <vector>
#include <sstream>
#include <math.h> 
#define PI M_PI


using namespace std;

// String tokenizer. From http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void Tokenize(const string& str,
                      vector<string>& tokens,
                      const string& delimiters = " ");

//Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
double angDiff(double end, double begin);

//Convert a integer to string
string intToStr(int integer);

//Saturate  a vector to a limit modulo, keeping scale.
void saturation(double &x, double &y, double limit);

//Calculates mean and variance iteratively. New mean and variance are return by reference in mean and var. 
// From https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
void IterativeMeanVariance(double x, unsigned long n, double* mean, double* var);

#endif
