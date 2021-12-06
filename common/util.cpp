#include "util.h"

// String tokenizer. From http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void Tokenize(const string& str,
                      vector<string>& tokens,
                      const string& delimiters)
{
    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

//Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
double angDiff(double end, double begin)
{
   double returnMe = end - begin;
  
   if (returnMe > PI)
      returnMe = -(2*PI - returnMe);
   else if (returnMe < -PI)
      returnMe = 2*PI + returnMe;
    
   return returnMe;
}


//Convert a integer to string
string intToStr(int integer)
{
   ostringstream stringNumero;

   stringNumero << integer;

   return stringNumero.str();
}

//Saturate  a vector to a limit modulo, keeping scale.
void saturation(double &x, double &y, double limit)
{
   double factor;

   //Saturation (keeping scale between x and y)
   if ( (x > limit) || (y > limit) )
   {
      if (x > y)
      {
         factor = x / limit;
         x = x / factor;
         y = y / factor;
      }
      else
      {
         factor = y / limit;
         y = y / factor;
         x = x / factor;
      }
   }
   if ( (x < -limit) || (y < -limit) )
   {
      if (x < y)
      {
         factor = x / -limit;
         x = x / factor;
         y = y / factor;
      }
      else
      {
         factor = y / -limit;
         y = y / factor;
         x = x / factor;
      }
   }
}

//Calculates mean and variance iteratively. New mean and variance are return by reference in mean and var. 
// From https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
void IterativeMeanVariance(double x, unsigned long n, double* mean, double* var){
  double mean_old = *mean, var_old = *var;
  *mean = mean_old + (x - mean_old)/n;
  *var = var_old + mean_old*mean_old - (*mean)*(*mean) + (x*x - var_old - mean_old*mean_old)/n;
}
