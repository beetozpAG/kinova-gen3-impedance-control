#ifndef MATRIX_H
#define MATRIX_H

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <type_traits>

using std::array;
using std::common_type;
using std::initializer_list;
using std::size_t;

namespace matrix
{

	template <typename T, size_t N, size_t M>
	struct Matrix
	{
		array<array<T, M>, N> data{};

		Matrix(const Matrix &) = default;
		Matrix(Matrix &&) = default;
		Matrix &operator=(const Matrix &) = default;
		Matrix &operator=(Matrix &&) = default;

		Matrix()
		{
		}
		Matrix(initializer_list<T> l)
		{
			assert(l.size() <= M * N);
			auto c = l.begin();
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					data[i][j] = *(c++);
				}
			}
		}

		array<T, M> &operator[](size_t i)
		{
			return data[i];
		};
		const array<T, M> &operator[](size_t i) const
		{
			return data[i];
		};

		template <typename iT, size_t O>
		Matrix<typename common_type<T, iT>::type, N, O>
		operator*(const Matrix<iT, M, O> &other) const
		{
			Matrix<typename common_type<T, iT>::type, N, O> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < O; j++)
				{
					for (size_t k = 0; k < M; k++)
					{
						res[i][j] += data[i][k] * other[k][j];
					}
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, M>
		hadamard(const Matrix<iT, N, M> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, M> res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[i][j] = data[i][j] * other[i][j];
				}
			}
			return res;
		}

		Matrix<T, M, N> trans()
		{
			Matrix<T, M, N> res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[j][i] = data[i][j];
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, M>
		operator+(const Matrix<iT, N, M> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, M> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[i][j] = data[i][j] + other[i][j];
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, M>
		operator-(const Matrix<iT, N, M> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, M> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[i][j] = data[i][j] - other[i][j];
				}
			}
			return res;
		}

		Matrix operator-() const
		{
			Matrix res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[i][j] = -data[i][j];
				}
			}
			return res;
		}
		template <size_t nS, size_t nE, size_t mS, size_t mE>
		Matrix<T, nE - nS, mE - mS> select() const
		{
			Matrix<T, nE - nS, mE - mS> res{};
			for (size_t i = nS; i < nE; i++)
			{
				for (size_t j = mS; j < mE; j++)
				{
					res[i - nS][j - mS] = data[i][j];
				}
			}
			return res;
		}

		static Matrix zeros()
		{
			return {};
		}
		
		static Matrix ones()
		{
			Matrix res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < M; j++)
				{
					res[i][j] = 1;
				}
			}
			return res;
		}
	};

	template <typename T, size_t N>
	struct Matrix<T, N, N>
	{
		array<array<T, N>, N> data{};

		Matrix(const Matrix &) = default;
		Matrix(Matrix &&) = default;
		Matrix &operator=(const Matrix &) = default;
		Matrix &operator=(Matrix &&) = default;

		Matrix()
		{
		}
		Matrix(initializer_list<T> l)
		{
			assert(l.size() <= N * N);
			auto c = l.begin();
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					data[i][j] = *(c++);
				}
			}
		}

		array<T, N> &operator[](size_t i)
		{
			return data[i];
		};
		const array<T, N> &operator[](size_t i) const
		{
			return data[i];
		};

		template <typename iT, size_t O>
		Matrix<typename common_type<T, iT>::type, N, O>
		operator*(const Matrix<iT, N, O> &other) const
		{
			Matrix<typename common_type<T, iT>::type, N, O> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < O; j++)
				{
					for (size_t k = 0; k < N; k++)
					{
						res[i][j] += data[i][k] * other[k][j];
					}
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, N>
		hadamard(const Matrix<iT, N, N> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, N> res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[i][j] = data[i][j] * other[i][j];
				}
			}
			return res;
		}

		Matrix<T, N, N> trans()
		{
			Matrix<T, N, N> res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[j][i] = data[i][j];
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, N>
		operator+(const Matrix<iT, N, N> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, N> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[i][j] = data[i][j] + other[i][j];
				}
			}
			return res;
		}

		template <typename iT>
		Matrix<typename common_type<T, iT>::type, N, N>
		operator-(const Matrix<iT, N, N> &other) const
		{
			Matrix<typename common_type<iT, T>::type, N, N> res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[i][j] = data[i][j] - other[i][j];
				}
			}
			return res;
		}

		Matrix operator-() const
		{
			Matrix res;
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[i][j] = -data[i][j];
				}
			}
			return res;
		}
		template <size_t nS, size_t nE, size_t mS, size_t mE>
		Matrix<T, nE - nS, mE - mS> select() const
		{
			Matrix<T, nE - nS, mE - mS> res{};
			for (size_t i = nS; i < nE; i++)
			{
				for (size_t j = mS; j < mE; j++)
				{
					res[i - nS][j - mS] = data[i][j];
				}
			}
			return res;
		}
		static Matrix diagonal(T t)
		{
			Matrix res{};
			for (size_t i = 0; i < N; i++)
			{
				res[i][i] = t;
			}
			return res;
		}

		static Matrix identity()
		{
			return diagonal(1);
		}

		static Matrix zeros()
		{
			return {};
		}

		static Matrix ones()
		{
			Matrix res{};
			for (size_t i = 0; i < N; i++)
			{
				for (size_t j = 0; j < N; j++)
				{
					res[i][j] = 1;
				}
			}
			return res;
		}
	};

	template <typename CharT, typename Traits, typename T, size_t N, size_t M>
	std::basic_ostream<CharT, Traits> &
	operator<<(std::basic_ostream<CharT, Traits> &ostream, Matrix<T, N, M> m)
	{
		for (size_t i = 0; i < N - 1; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				ostream << m[i][j] << ", ";
			}
		}
		for (size_t j = 0; j < M - 1; j++)
		{
			ostream << m[N - 1][j] << ", ";
		}
		ostream << m[N - 1][M - 1];
		return ostream;
	}

	template <typename T, size_t N>
	Matrix<T, N, N> identity()
	{
		Matrix<T, N, N> res{};
		for (size_t i = 0; i < N; i++)
		{
			res[i][i] = 1;
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> zeros()
	{
		return {};
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, M, N> trans(const Matrix<T, N, M> &m)
	{
		Matrix<T, M, N> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[j][i] = m[i][j];
			}
		}
		return res;
	}

	template <typename uT, typename lT, size_t N1, size_t N2, size_t M>
	Matrix<typename common_type<uT, lT>::type, N1 + N2, M>
	cV(const Matrix<uT, N1, M> &u, const Matrix<lT, N2, M> &l)
	{
		Matrix<typename std::common_type<uT, lT>::type, N1 + N2, M> res{};
		for (size_t i = 0; i < N1; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = u[i][j];
			}
		}
		for (size_t i = 0; i < N2; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i + N1][j] = l[i][j];
			}
		}
		return res;
	}

	template <typename uT, typename... lT>
	auto cV(uT u, lT... args) -> decltype(cV(u, cV(args...)))
	{
		return cV(u, cV(args...));
	}

	template <typename lT, typename rT, size_t N, size_t M1, size_t M2>
	Matrix<typename common_type<lT, rT>::type, N, M1 + M2>
	cH(const Matrix<lT, N, M1> &l, const Matrix<rT, N, M2> &r)
	{
		Matrix<typename common_type<lT, rT>::type, N, M1 + M2> res;
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M1; j++)
			{
				res[i][j] = l[i][j];
			}
		}
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M2; j++)
			{
				res[i][j + M1] = r[i][j];
			}
		}
		return res;
	}

	template <typename lT, typename... rT>
	auto cH(lT l, rT... args) -> decltype(cH(l, cH(args...)))
	{
		return cH(l, cH(args...));
	}

	template <typename aT, typename bT, size_t N, size_t M>
	Matrix<typename common_type<aT, bT>::type, N, M>
	operator*(const Matrix<aT, N, M> &a, bT b)
	{
		Matrix<typename common_type<aT, bT>::type, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = a[i][j] * b;
			}
		}
		return res;
	}

	template <typename aT, typename bT, size_t N, size_t M>
	Matrix<typename common_type<aT, bT>::type, N, M>
	operator*(aT a, const Matrix<bT, N, M> &b)
	{
		Matrix<typename common_type<aT, bT>::type, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = a * b[i][j];
			}
		}
		return res;
	}

	template <typename aT, typename bT, size_t N, size_t M>
	Matrix<typename common_type<aT, bT>::type, N, M>
	operator/(const Matrix<aT, N, M> &a, bT b)
	{
		Matrix<typename common_type<aT, bT>::type, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = a[i][j] / b;
			}
		}
		return res;
	}

	template <typename aT, typename bT, size_t N, size_t M>
	Matrix<typename common_type<aT, bT>::type, N, M>
	operator/(aT a, const Matrix<bT, N, M> &b)
	{
		Matrix<typename common_type<aT, bT>::type, N, M> res;
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = a / b[i][j];
			}
		}
		return res;
	}

	template <typename T, size_t N>
	T trace(const Matrix<T, N, N> &m)
	{
		T res{};
		for (size_t i = 0; i < N; i++)
		{
			res += m[i][i];
		}
		return res;
	}

	template <typename T>
	Matrix<T, 3, 3> skew(const Matrix<T, 3, 1> &m)
	{
		Matrix<T, 3, 3> res{};
		res[0][1] = -m[2][0];
		res[0][2] = m[1][0];
		res[1][0] = m[2][0];
		res[1][2] = -m[0][0];
		res[2][0] = -m[1][0];
		res[2][1] = m[0][0];
		return res;
	}

	template <size_t nS, size_t nE, size_t mS, size_t mE, typename T, size_t N,
			  size_t M>
	Matrix<T, nE - nS, mE - mS> select(const Matrix<T, N, M> &m)
	{
		Matrix<T, nE - nS, mE - mS> res{};
		for (size_t i = nS; i < nE; i++)
		{
			for (size_t j = mS; j < mE; j++)
			{
				res[i - nS][j - mS] = m[i][j];
			}
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> tanh(const Matrix<T, N, M> &m)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = std::tanh(m[i][j]);
			}
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> abs(const Matrix<T, N, M> &m)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = std::abs(m[i][j]);
			}
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> mpow(const Matrix<T, N, M> &m, const float p)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = std::pow(m[i][j], p);
			}
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> sign(const Matrix<T, N, M> &m)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = m[i][j] == 0 ? 0 : m[i][j] / std::abs(m[i][j]);
			}
		}
		return res;
	}

	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> smoothSign(const Matrix<T, N, M> &m, T factor)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = std::tanh(factor * pow(m[i][j], 3));
			}
		}
		return res;
	}


	template <typename T, size_t N, size_t M>
	Matrix<T, N, M> exp(const Matrix<T, N, M> &m)
	{
		Matrix<T, N, M> res{};
		for (size_t i = 0; i < N; i++)
		{
			for (size_t j = 0; j < M; j++)
			{
				res[i][j] = std::exp(m[i][j]);
			}
		}
		return res;
	}
	
	template <size_t N, size_t M> using mat = Matrix<float, N, M>;
	template <size_t N> using vec           = Matrix<float, N, 1>;


} // namespace matrix
#endif // #ifndef MATRIX_H
