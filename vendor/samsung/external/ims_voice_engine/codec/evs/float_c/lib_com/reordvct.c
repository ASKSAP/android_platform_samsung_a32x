/*====================================================================================
    EVS Codec 3GPP TS26.443 Oct 20, 2015. Version 12.4.0
  ====================================================================================*/

#include "options.h"
#include "prot.h"

/*--------------------------------------------------------------------------*
 * reordvct()
 *
 * Rearrange a vector in decreasing order
 *--------------------------------------------------------------------------*/

void reordvct(
    short *y,         /* i/o: vector to rearrange    */
    short N,          /* i  : dimensions             */
    short *idx        /* o  : reordered vector index */
)
{
    short i, j, k, n, im, temp;

    n = N - 1;
    for (i=0; i<n; i++)
    {
        im = i;
        k = i + 1;
        for (j=k; j<N; j++)
        {
            if (y[im] < y[j])
            {
                im = j;
            }
        }

        temp = y[i];
        y[i] = y[im];
        y[im] = temp;
        j = idx[i];
        idx[i] = idx[im];
        idx[im] = j;
    }

    return;
}
