/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include "common.h"

bool isdigit(unsigned char ch)
{
  if (ch >= '0' && ch <= '9')  return true;
  return false;
}

unsigned char fromdigitch(unsigned char ch)
{
  if (ch >= '0' && ch <= '9')  return (ch - '0');
  return 0;
}

//check if character is valid hex digit
bool ishexch(unsigned char ch)
{
  if (ch >= '0' && ch <= '9')  return true;
  if (ch >= 'a' && ch <= 'f')  return true;
  if (ch >= 'A' && ch <= 'F')  return true;
  return false;
}

//Convert hex char to uint
unsigned char fromhexch(unsigned char ch)
{
  if (ch >= '0' && ch <= '9')  return (ch - '0');
  if (ch >= 'a' && ch <= 'f')  return (ch - 'a' + 10);
  if (ch >= 'A' && ch <= 'F')  return (ch - 'A' + 10);
  return 0;
}
