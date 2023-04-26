#ifndef MATRIX_H
#define MATRIX_H

#include <string>
#include <vector>
#include <ostream>

template<int R, int C>
class Matrix
{
private:
    //size_t _rowCount;
    //size_t _columnCount;
    double  _data[R][C];

public:
    // constructor:
    Matrix()
    {
        // always initialize Matrix to zero
        (*this) *= 0.0;   // should only work if "operator*" is defined properly below
    }

    // (volatile) element access:
    double& operator() (int m, int n)
    {
        return _data[m][n];
    }

    // scalar addition "+":
    Matrix<R, C> operator+(const double scale) const
    {
        Matrix<R, C> newMat;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                newMat._data[i][j] = _data[i][j] + scale;
            }
        }
        return newMat;
    }

    // scalar addition "+":
    void operator+=(const double scale)
    {
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                _data[i][j] += scale;
            }
        }
    }

    // matrix addition "+":
    Matrix<R, C> operator+(const Matrix<R, C> &otherMat) const
    {
        Matrix<R, C> newMat;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                newMat._data[i][j] = _data[i][j] + otherMat._data[i][j];
            }
        }
        return newMat;
    }

    // matrix addition "+="
    void operator+=(const Matrix<R, C> &otherMat)
    {
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                _data[i][j] += otherMat._data[i][j];
            }
        } 
    }

    // scalar multiplication behind (i.e. Matrix*Scalar):
    Matrix<R, C> operator*(const double scale) const
    {
        Matrix<R, C> newMat;
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                newMat._data[i][j] = _data[i][j] * scale;
            }
        }
        return newMat;
    }

    // scalar multiplication in front (i.e. Scalar*Matrix)
    friend Matrix<R, C> operator*(const double scale, const Matrix<R, C> &otherMat)
    {
        return (otherMat * scale);  // just re-use the operator defined above
    }

    // scalar multiplication on itself (i.e. Matrix*=scalar)
    void operator*=(const double scale)
    {
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                _data[i][j] *= scale;
            }
        }
    }

    // matrix multiplication:
    template <int newC>
    Matrix<R, newC> operator*(const Matrix<C, newC> &otherMat) const
    {
        Matrix<R, newC> newMat; // requires constructor to properly initialize matrix to zero
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < newC; j++)
            {
                for (int k = 0; k < C; k++)
                {
                    newMat(i,j) += ( _data[i][k] * otherMat.getElem(k,j) ); // need to use getElem here b/c we're contracted to keep otherMat const
                }
            }
        }
        return newMat;
    }

    // function to fetch the number of rows:
    int numRows() { return sizeof(_data) / sizeof(_data[0]); }

    // function to fetch the number of columns:
    int numCols() { return sizeof(_data[0]) / sizeof(double); }

    // (secure) element access:
    double getElem(int m, int n) const { return _data[m][n]; }

    // (secure) element setting:
    void setelem(int m, int n, const double val) { _data[m][n] = val; }

    // function to create string of matrix for printing:
    std::string toString()
    {
        std::string outstr = "";
        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                outstr += std::to_string(_data[i][j]) + "\t";
            }
            outstr += "\n";
        }
        return outstr;
    }

};

#endif