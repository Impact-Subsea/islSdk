#ifndef EIGEN_H_
#define EIGEN_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/maths.h"
#include "maths/matrix.h"
#include "maths/vector.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Math
    {
        namespace Eigen
        {
			template<uint_t Rows, uint_t Cols>
			void computeSymmetric(Matrix<Rows, Cols>& mat, Vector<Rows>& eigVal, Matrix<Rows, Cols>& eigVec)
			{
				Matrix A = mat;
				const int_t n = Rows;

				for (int_t r = 0; r < n; r++)
				{
					for (int_t c = 0; c < n; c++)
					{
						eigVec[r][c] = 0.0;
					}

					eigVec[r][r] = 1.0;
					eigVal[r] = A[r][r];
				}

				uint_t maxLoops = 15;
				real_t residue = 0.0;
				do
				{
					for (int_t r = 0; r < n - 1; r++)
					{
						for (int_t c = r + 1; c < n; c++)
						{
							residue += Math::abs(A[r][c]);
						}
					}

					if (residue > 0.0)
					{
						for (int_t r = 0; r < n - 1; r++)
						{
							for (int_t c = r + 1; c < n; c++)
							{
								if (Math::abs(A[r][c]) > 0.0)
								{
									real_t cot2phi = 0.5 * (eigVal[c] - eigVal[r]) / (A[r][c]);
									real_t tanphi = 1.0 / (Math::abs(cot2phi) + Math::sqrt(1.0 + cot2phi * cot2phi));
									if (cot2phi < 0.0)
									{
										tanphi = -tanphi;
									}

									real_t cosphi = 1.0 / Math::sqrt(1.0 + tanphi * tanphi);
									real_t sinphi = tanphi * cosphi;
									real_t tanhalfphi = sinphi / (1.0 + cosphi);
									real_t tmp = tanphi * A[r][c];
									eigVal[r] -= tmp;
									eigVal[c] += tmp;
									A[r][c] = 0.0;

									for (int_t j = 0; j < n; j++)
									{
										tmp = eigVec[j][r];
										eigVec[j][r] = tmp - sinphi * (eigVec[j][c] + tanhalfphi * tmp);
										eigVec[j][c] = eigVec[j][c] + sinphi * (tmp - tanhalfphi * eigVec[j][c]);
									}

									for (int_t j = 0; j <= r - 1; j++)
									{
										tmp = A[j][r];
										A[j][r] = tmp - sinphi * (A[j][c] + tanhalfphi * tmp);
										A[j][c] = A[j][c] + sinphi * (tmp - tanhalfphi * A[j][c]);
									}
									for (int_t j = r + 1; j <= c - 1; j++)
									{
										tmp = A[r][j];
										A[r][j] = tmp - sinphi * (A[j][c] + tanhalfphi * tmp);
										A[j][c] = A[j][c] + sinphi * (tmp - tanhalfphi * A[j][c]);
									}
									for (int_t j = c + 1; j < n; j++)
									{
										tmp = A[r][j];
										A[r][j] = tmp - sinphi * (A[c][j] + tanhalfphi * tmp);
										A[c][j] = A[c][j] + sinphi * (tmp - tanhalfphi * A[c][j]);
									}
								}
							}
						}
					}
				} while ((residue > 0) && maxLoops--);
			}
        };
    }
}
//--------------------------------------------------------------------------------------------------
#endif
