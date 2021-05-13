/*====================================================================================
    EVS Codec 3GPP TS26.443 Oct 20, 2015. Version 12.4.0
  ====================================================================================*/

/*
  ===========================================================================
   File: TYPEDEF.H                                       v.2.3 - 30.Nov.2009
  ===========================================================================

            ITU-T STL  BASIC OPERATORS

            TYPE DEFINITION PROTOTYPES

   History:
   26.Jan.00   v1.0     Incorporated to the STL from updated G.723.1/G.729
                        basic operator library (based on basic_op.h)

   03 Nov 04   v2.0     Incorporation of new 32-bit / 40-bit / control
                        operators for the ITU-T Standard Tool Library as
                        described in Geneva, 20-30 January 2004 WP 3/16 Q10/16
                        TD 11 document and subsequent discussions on the
                        wp3audio@yahoogroups.com email reflector.
   March 06   v2.1      Changed to improve portability.

  ============================================================================
*/

/*_____________________
 |                     |
 | Basic types.        |
 |_____________________|
*/

#include "options.h"

#ifndef TYPEDEF_H
#define TYPEDEF_H

/*
 * This is the original code from the file typedef.h
 */
#if defined(__BORLANDC__) || defined(__WATCOMC__) || defined(_MSC_VER) || defined(__ZTC__)
typedef signed char Word8;
typedef unsigned char UWord8;
typedef short Word16;
typedef int Word32;
typedef unsigned short UWord16;
typedef unsigned int UWord32;
typedef __int64 Word40;
typedef int Flag;

#elif defined(__CYGWIN__)
typedef signed char Word8;
typedef unsigned char UWord8;
typedef short Word16;
typedef int Word32;
typedef unsigned short UWord16;
typedef unsigned int UWord32;
typedef long long Word40;
typedef int Flag;

#elif defined(__sun)
typedef signed char Word8;
typedef unsigned char UWord8;
typedef short Word16;
typedef long Word32;
/*#error "The 40-bit operations have not been tested on __sun : need to define Word40"*/
typedef unsigned short UWord16;
typedef unsigned long UWord32;
typedef long long Word40;
typedef int Flag;

#elif defined(__unix__) || defined(__unix) || defined(__APPLE__)
typedef signed char Word8;
typedef unsigned char UWord8;
typedef short Word16;
typedef int Word32;
typedef unsigned short UWord16;
typedef unsigned int UWord32;
/*#error "The 40-bit operations have not been tested on unix : need to define Word40"*/
typedef long long Word40;
typedef int Flag;
#endif

typedef float Float32;

#endif /* ifndef _TYPEDEF_H */


/* end of file */




