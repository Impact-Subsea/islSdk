#ifndef VECTOR_H_
#define VECTOR_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/maths.h"
#include <type_traits>
#include <ostream>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Math
    {
        // Helper struct to conditionally include named members
        template <uint_t Length, typename Enable = void>
        struct VectorStorage
        {
            real_t m_data[Length];
        };

        // Specialization for Length == 2
        template <uint_t Length>
        struct VectorStorage<Length, typename std::enable_if<Length == 2>::type>
        {
            union
            {
                real_t m_data[Length];
                struct { real_t x, y; };
            };
        };

        // Specialization for Length == 3
        template <uint_t Length>
        struct VectorStorage<Length, typename std::enable_if<Length == 3>::type>
        {
            union
            {
                real_t m_data[Length];
                struct { real_t x, y, z; };
            };
        };

        // Specialization for Length == 4
        template <uint_t Length>
        struct VectorStorage<Length, typename std::enable_if<Length == 4>::type>
        {
            union
            {
                real_t m_data[Length];
                struct { real_t w, x, y, z; };
            };
        };

        template <uint_t Length>
        class Vector : public VectorStorage<Length>
        {
        public:
            Vector() : VectorStorage<Length>{} {}

            template <typename... Args, typename = typename std::enable_if<sizeof...(Args) == Length>::type>
            Vector(Args... args) : VectorStorage<Length>{ static_cast<real_t>(args)... }
            {
                static_assert((std::is_convertible<Args, real_t>::value && ...), "All arguments must be convertible to real_t");
            }

            real_t& operator[](uint_t index)
            {
                return this->m_data[index];
            }

            const real_t& operator[](uint_t index) const
            {
                return this->m_data[index];
            }

            bool_t operator==(const Vector<Length>& other) const
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    if (this->m_data[i] != other.m_data[i])
                    {
                        return false;
                    }
                }
                return true;
            }

            bool_t operator!=(const Vector<Length>& other) const
            {
                return !(*this == other);
            }

            Vector<Length> operator-() const
            {
                Vector<Length> result;
                for (uint_t i = 0; i < Length; i++)
                {
                    result.m_data[i] = -this->m_data[i];
                }
                return result;
            }

            Vector<Length>& operator+=(const Vector<Length>& other)
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    this->m_data[i] += other.m_data[i];
                }
                return *this;
            }

            Vector<Length>& operator-=(const Vector<Length>& other)
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    this->m_data[i] -= other.m_data[i];
                }
                return *this;
            }

            Vector<Length>& operator*=(const real_t scaler)
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    this->m_data[i] *= scaler;
                }
                return *this;
            }

            Vector<Length> operator+(const Vector<Length>& other) const
            {
                Vector<Length> result;
                for (uint_t i = 0; i < Length; i++)
                {
                    result[i] = this->m_data[i] + other.m_data[i];
                }
                return result;
            }

            Vector<Length> operator-(const Vector<Length>& other) const
            {
                Vector<Length> result;
                for (uint_t i = 0; i < Length; i++)
                {
                    result[i] = this->m_data[i] - other.m_data[i];
                }
                return result;
            }

            Vector<Length> operator*(const real_t scaler) const
            {
                Vector<Length> result;
                for (uint_t i = 0; i < Length; i++)
                {
                    result[i] = this->m_data[i] * scaler;
                }
                return result;
            }

            void zero()
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    this->m_data[i] = 0;
                }
            }

            void fill(real_t value)
            {
                for (uint_t i = 0; i < Length; i++)
                {
                    this->m_data[i] = value;
                }
            }

            real_t magnitude() const
            {
                return Math::sqrt(magnitudeSq());
            }

            real_t magnitudeSq() const
            {
                real_t magnitudeSq = 0;

                for (size_t i = 0; i < Length; i++)
                {
                    magnitudeSq += this->m_data[i] * this->m_data[i];
                }
                return magnitudeSq;
            }

            Vector<Length> normalise() const
            {
                Vector<Length> result;
                real_t mag = magnitude();

                if (mag != (real_t)0.0)
                {
                    mag = (real_t)1.0 / mag;
                }

                for (uint_t i = 0; i < Length; i++)
                {
                    result.m_data[i] = this->m_data[i] * mag;
                }
                return result;
            }

            real_t dot(const Vector<Length>& other) const
            {
                real_t dotProduct = 0;

                for (uint_t i = 0; i < Length; i++)
                {
                    dotProduct += this->m_data[i] * other.m_data[i];
                }
                return dotProduct;
            }
        };

        template <uint_t Length>
        std::ostream& operator<<(std::ostream& os, const Vector<Length>& vec)
        {
            os << "(";
            for (uint_t i = 0; i < Length; ++i)
            {
                os << vec[i];
                if (i < Length - 1)
                {
                    os << ", ";
                }
            }
            os << ")";
            return os;
        }

        class Vector2 : public Vector<2>
        {
        public:
      
            Vector2() : Vector<2>() {}
            Vector2(real_t x, real_t y) : Vector<2>(x, y) {}
            Vector2(const Vector<2>& vec) : Vector<2>(vec[0], vec[1]) {}
        };

        class Vector3 : public Vector<3>
        {
        public:
            enum class Axis { xPlus, xMinus, yPlus, yMinus, zPlus, zMinus };

            Vector3() : Vector<3>() {}
            Vector3(real_t x, real_t y, real_t z) : Vector<3>(x, y, z) {}
            Vector3(const Vector<3>& vec) : Vector<3>(vec[0], vec[1], vec[2]) {}
            Vector3 cross(const Vector<3>& other) const;
            Vector3::Axis findClosestCardinalAxis() const;
            static Vector3 getVectorFromAxis(Vector3::Axis axis);
        };
    }
}
//--------------------------------------------------------------------------------------------------
#endif
