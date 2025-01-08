#ifndef MATRIX_H_
#define MATRIX_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/maths.h"
#include "maths/vector.h"
#include <ostream>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Math
    {
        template <uint_t Rows, uint_t Cols>
        class Matrix
        {
        private:
            bool_t choleskyDecompose(Matrix<Rows, Cols>& result, const Matrix<Rows, Cols>& m) const
            {
                result.fill(0);

                for (int_t i = 0; i < static_cast<int_t>(Cols); i++)
                {
                    for (int_t j = i; j < static_cast<int_t>(Cols); j++)
                    {
                        real_t sum = m[j][i];
                        for (int_t k = i - 1; k >= 0; k--)
                        {
                            sum -= result[k][i] * result[k][j];
                        }
                        if (i == j)
                        {
                            if (sum <= 0.0)
                            {
                                return false;
                            }
                            result[i][i] = Math::sqrt(sum);
                        }
                        else
                        {
                            result[i][j] = sum / result[i][i];
                        }
                    }
                }
                return true;
            }

        protected:
            real_t m_data[Rows][Cols];

        public:
            Matrix() : m_data{} {}

            template <typename... Args, typename = typename std::enable_if<sizeof...(Args) == Rows * Cols >::type>
            Matrix(Args... args) : m_data{ static_cast<real_t>(args)... }
            {
                static_assert((std::is_convertible<Args, real_t>::value && ...), "All arguments must be convertible to real_t");
            }

            Matrix(const Matrix<Rows, Cols>& other)
			{
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
						m_data[i][j] = other.m_data[i][j];
					}
                }
			}

            real_t* operator[](uint_t index)
            {
                return &m_data[index][0];
            }

            const real_t* operator[](uint_t index) const
            {
                return &m_data[index][0];
            }

            bool_t operator==(const Matrix<Rows, Cols>& other) const
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        if (m_data[i][j] != other.m_data[i][j])
                        {
                            return false;
                        }
                    }
                }
                return true;
            }

            bool_t operator!=(const Matrix<Rows, Cols>& other) const
            {
                return !(*this == other);
            }

            Matrix<Rows, Cols> operator-() const
            {
                Matrix<Rows, Cols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result.m_data[i][j] = -m_data[i][j];
                    }
                }
                return result;
            }

            Matrix<Rows, Cols>& operator+=(const Matrix<Rows, Cols>& other)
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m_data[i][j] += other.m_data[i][j];
                    }
                }
                return *this;
            }

            Matrix<Rows, Cols>& operator-=(const Matrix<Rows, Cols>& other)
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m_data[i][j] -= other.m_data[i][j];
                    }
                }
                return *this;
            }

            Matrix<Rows, Cols>& operator*=(const real_t scaler)
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m_data[i][j] *= scaler;
                    }
                }
                return *this;
            }

            Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols>& other) const
            {
                Matrix<Rows, Cols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result.m_data[i][j] = m_data[i][j] + other.m_data[i][j];
                    }
                }
                return result;
            }

            Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols>& other) const
            {
                Matrix<Rows, Cols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result.m_data[i][j] = m_data[i][j] - other.m_data[i][j];
                    }
                }
                return result;
            }

            Matrix<Rows, Cols> operator*(const real_t scaler) const
            {
                Matrix<Rows, Cols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result.m_data[i][j] = m_data[i][j] + scaler;
                    }
                }
                return result;
            }

            template <uint_t OtherCols>
            Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& other) const
            {
                Matrix<Rows, OtherCols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < OtherCols; j++)
                    {
                        for (uint_t k = 0; k < Cols; k++)
                        {
                            result.m_data[i][j] += m_data[i][k] * other.m_data[k][j];
                        }
                    }
                }
                return result;
            }

            Vector<Rows> operator*(const Vector<Cols>& vec) const
            {
                Vector<Rows> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result[i] += m_data[i][j] * vec[j];
                    }
                }
                return result;
            }

            Matrix<Cols, Cols> multiplyByTranspose() const
            {
                Matrix<Cols, Cols> result;
                for (uint_t i = 0; i < Cols; i++)
                {
                    for (uint_t j = i; j < Cols; j++)
                    {
                        for (uint_t k = 0; k < Rows; k++)
                        {
                            result.m_data[i][j] += m_data[k][i] * m_data[k][j];
                        }
                        result.m_data[j][i] = result.m_data[i][j];
                    }
                }
                return result;
            }

            static Matrix<Rows, Cols> identity()
            {
                Matrix<Rows, Cols> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result[i][j] = i == j ? 1 : 0;
                    }
                }
                return result;
            }

            void setIdentity()
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m_data[i][j] = i == j ? 1 : 0;
                    }
                }
            }

            bool_t isSymmetric() const
            {
                if (Rows != Cols)
                {
                    return false;
                }

                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < i; j++)
                    {
                        if (m_data[i][j] != m_data[j][i])
                        {
                            return false;
                        }
                    }
                }
                return true;
            }

            void fill(real_t value)
            {
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m_data[i][j] = value;
                    }
                }
            }

            Matrix<Cols, Rows> transpose() const
            {
                Matrix<Cols, Rows> result;
                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        result.m_data[j][i] = m_data[i][j];
                    }
                }
                return result;
            }

            real_t determinant() const
            {
                if (Rows != Cols)
                {
                    return 0.0;
                }
                const uint_t N = Cols;
                real_t det = 1;
                real_t m[Rows][Cols];

                for (uint_t i = 0; i < Rows; i++)
                {
                    for (uint_t j = 0; j < Cols; j++)
                    {
                        m[i][j] = m_data[i][j];
                    }
                }

                for (uint_t i = 0; i < N - 1; i++)
                {
                    uint_t r = i;
                    real_t maxA = Math::abs(m[i][i]);
                    for (uint_t k = i + 1; k < N; k++)
                    {
                        real_t val = Math::abs(m[k][i]);
                        if (val > maxA)
                        {
                            r = k;
                            maxA = val;
                        }
                    }
                    if (r != i)
                    {
                        for (uint_t j = i; j < N; j++)
                        {
                            std::swap(m[i][j], m[r][j]);
                        }
                        det = -det;
                    }

                    real_t pivot = m[i][i];
                    if (Math::abs(pivot) < 1.0E-30) return 0.0;

                    for (uint_t r = i + 1; r < N; r++)
                    {
                        real_t multiple = m[r][i] / pivot;
                        for (uint_t j = i; j < N; j++)
                        {
                            m[r][j] -= multiple * m[i][j];
                        }
                    }
                    det *= pivot;
                }

                det *= m[N - 1][N - 1];

                return det;
            }

            Matrix<Rows, Cols> inverse() const
            {
                const int_t N = Cols;
                Matrix<Rows, Cols> inv;
                Matrix< Rows, Cols> L;
                bool_t symmetric = isSymmetric();

                inv.fill(0);

                // if input is not symetric -> A^-1 = A' (A A')^-1
                if (symmetric)
                {
                    if (!choleskyDecompose(L, *this))
                    {
                        return inv;
                    }
                }
                else
                {
                    if (!choleskyDecompose(L, (*this) * transpose()))
                    {
                        return inv;
                    }
                }
                // A = L L' -> L L' A^-1 = I -> A^-1 = I / L L' = I (L L')^-1
                for (int_t i = 0; i < N; i++)
                {
                    for (int_t j = 0; j <= i; j++)
                    {
                        real_t sum = i == j ? 1.0 : 0.0;
                        for (int_t k = i - 1; k >= j; k--)
                        {
                            sum -= L[k][i] * inv[k][j];
                        }
                        inv[i][j] = sum / L[i][i];
                    }
                }

                for (int_t i = N - 1; i >= 0; i--)
                {
                    for (int_t j = 0; j <= i; j++)
                    {
                        real_t sum = i < j ? 0.0 : inv[i][j];
                        for (int_t k = i + 1; k < N; k++)
                        {
                            sum -= L[i][k] * inv[k][j];
                        }
                        inv[j][i] = inv[i][j] = sum / L[i][i];
                    }
                }

                if (!symmetric)
                {
                    inv = transpose() * inv;
                }

                return inv;
            }
        };

        template <uint_t Rows, uint_t Cols>
        std::ostream& operator<<(std::ostream& os, const Matrix<Rows, Cols>& vec)
        {
            for (uint_t i = 0; i < Rows; ++i)
            {
                for (uint_t j = 0; j < Cols; ++j)
                {
                    os << vec[i][j];
                    if (j < Cols - 1)
                    {
                        os << "\t";
                    }
                }
                if (i < Rows - 1)
                {
                    os << "\n";
                }
            }
            return os;
        }

        class Matrix3x3 : public Matrix<3, 3>
        {
        public:
			Matrix3x3() : Matrix<3, 3>(1,0,0,0,1,0,0,0,1) {}
			Matrix3x3(real_t m00, real_t m01, real_t m02,
					  real_t m10, real_t m11, real_t m12,
					  real_t m20, real_t m21, real_t m22) : Matrix<3, 3>(m00, m01, m02, m10, m11, m12, m20, m21, m22) {}
			Matrix3x3(const Matrix<3, 3>& m) : Matrix<3, 3>(m) {}
            Matrix3x3(real_t heading, real_t pitch, real_t roll);

            static Matrix3x3 rodrigues(const Math::Vector3& axis, real_t cosAngle);
            static Matrix3x3 fromDownAndForward(const Math::Vector3& down, const Math::Vector3& forward);
            real_t determinant() const;
            Matrix3x3 inverseSymmetric() const;
        };
    }
}
//--------------------------------------------------------------------------------------------------
#endif
