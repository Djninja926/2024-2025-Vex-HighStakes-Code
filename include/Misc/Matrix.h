#pragma once
#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>

using namespace std;

class Matrix {
  private: 
    unsigned rowSize;
    unsigned colSize;
    vector<vector<double>> m_matrix;

  public:
  /* Constructors */
    // Default Empty Constructor
    Matrix() {}

    // Default Empty Destructor
    ~Matrix() {}

    // Default Non-Empty Constructor
    Matrix(unsigned m_rowSize, unsigned m_colSize, double initialVal = 0.0) {
      rowSize = m_rowSize;
      colSize = m_colSize;
      m_matrix.resize(rowSize);
      for (unsigned i = 0; i < m_matrix.size(); i++) {
        m_matrix[i].resize(m_colSize, initialVal);
      }
    }

    // Copy Constructer
    Matrix(const Matrix &M) {
      this-> rowSize = M.getRows();
      this-> colSize = M.getCols();
      this-> m_matrix = M.m_matrix;
    }

    // Copy Constructer
    Matrix(const vector<vector<double>> &mat) {
      this-> rowSize = mat.size();
      this-> colSize = mat[0].size();
      this-> m_matrix = mat;
    }

  /* Matrix Operations */
    void operator=(vector<vector<double>> B) {
      if (this->rowSize != B.size() || this->colSize != B[0].size()) {
        throw std::logic_error("Matrix and Vector<Vector<Double>> Dimensions Don't Match");
      }
      this-> rowSize = B.size();
      this-> colSize = B[0].size();
      this-> m_matrix = B;
    }

    Matrix operator+(Matrix B) {
      Matrix sum(rowSize, colSize);
      // Throw an error if the Dimensions don't match
      if (this-> rowSize != B.getRows() || this-> colSize != B.getCols()) {
        throw std::logic_error("Matrix Dimensions Don't Match");
      }
      
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          sum(i,j) = this-> m_matrix[i][j] + B(i,j);
        }
      }
      return sum;
    }

    Matrix operator-(Matrix B) {
      Matrix diff(rowSize, colSize);
      // Throw an error if the Dimensions don't match
      if (this-> rowSize != B.getRows() || this-> colSize != B.getCols()) {
        throw std::logic_error("Matrix Dimensions Don't Match");
      }
      
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          diff(i,j) = this-> m_matrix[i][j] - B(i,j);
        }
      }
      return diff;
    }

    Matrix operator*(Matrix B) {
      Matrix product(rowSize, B.getCols());
      // Throw an Error if Matrix #1 Columns don't match Matrix #2 Rows
      if (colSize != B.getRows()) {
        throw std::logic_error("Columns of Matrix #1 != Rows of Matrix #2. Suggested Fix: Swap Multiplication Order");
      }

      double temp = 0.0;

      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < B.getCols(); j++) {
          temp = 0.0;
          for (unsigned k = 0; k < colSize; k++) {
            temp += m_matrix[i][k] * B(k, j);
          }
          product(i, j) = temp;
        }
      }
      return product;
    }
    
  /* Scalar Operations */
    Matrix operator+(double Scalar) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = this-> m_matrix[i][j] + Scalar;
        }
      }
      return result;
    }

    Matrix operator-(double Scalar) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = this-> m_matrix[i][j] - Scalar;
        }
      }
      return result;
    }

    Matrix operator*(double Scalar) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = this-> m_matrix[i][j] * Scalar;
        }
      }
      return result;
    }

    Matrix operator/(double Scalar) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = this-> m_matrix[i][j] / Scalar;
        }
      }
      return result;
    }

    Matrix operator^(double B) {
      for (int X = 0; X < B; X++) {
        *this = (*this) * (*this);
      }
      return *this;
    }


  /* Aesthetic Methods */

    // Returns Value of Given Location When Asked in the Form A(X, Y)
    double& operator()(const unsigned &rowNo, const unsigned &colNo) {
      return this-> m_matrix[rowNo][colNo];
    }

    vector<double>& operator[](const unsigned num) {
      return this-> m_matrix[num];
    }

    // Returns the # of Rows
    unsigned getRows() const {
      return this-> rowSize;
    }

    // Returns the # of Columns
    unsigned getCols() const {
      return this-> colSize;
    }

    // Prints out the Matrix
    void PrintMat() const {
      cout << "Matrix:" << endl;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
            cout << "[" << m_matrix[i][j] << "] ";
          }
        cout << endl;
      }
    }

  /* Comparator Methods */
    bool operator==(Matrix&B) {
      if (rowSize == B.getRows() && colSize == B.getCols()) {
        for (int i = 0; i < rowSize; i++) {
          for (int j = 0; j < colSize; j++) {
            if (m_matrix[i][j] != B(i, j)) {
              return false;
            }
          }
        }
        return true;
      }
      return false;
    }

    bool operator!=(Matrix&B) {
      if (rowSize == B.getRows() && colSize == B.getCols()) {
        for (int i = 0; i < rowSize; i++) {
          for (int j = 0; j < colSize; j++) {
            if (m_matrix[i][j] != B(i, j)) {
              return true;
            }
          }
        }
        return false;
      }
      return true;
    }

    // Matrix
    Matrix operator+=(Matrix B) {
      *this = *this + B;
      return *this;
    }

    Matrix operator-=(Matrix B) {
      *this = *this - B;
      return *this;
    } 

    Matrix operator*=(Matrix B) {
      *this = *this * B;
      return *this;
    }
    
    // Scalar
    Matrix operator+=(double B) {
      *this = *this + B;
      return *this;
    }

    Matrix operator-=(double B) {
      *this = *this - B;
      return *this;
    } 

    Matrix operator*=(double B) {
      *this = *this * B;
      return *this;
    }

    Matrix operator/=(double B) {
      *this = *this / B;
      return *this;
    }

  /* Special Methods */

    double sum(void) {
      double sum = 0;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          sum += this-> m_matrix[i][j];
        }
      }
      return sum;
    }

    double prod(void) {
      double product = 0;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          product *= this-> m_matrix[i][j];
        }
      }
      return product;
    }

    Matrix square(void) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = this-> m_matrix[i][j] * this-> m_matrix[i][j];
        }
      }
      return result;
    }

    Matrix sqrt(void) {
      Matrix result(rowSize, colSize);
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          result(i,j) = std::sqrt(this->m_matrix[i][j]);
        }
      }
      return result;
    }

  Matrix cholesky(void) {
    if (rowSize != colSize) {
      throw std::logic_error("Cholesky decomposition requires square matrix.");
    }
    const int n = rowSize;

    // Initialize lower triangular matrix with zeros
    Matrix L(n, n, 0.0);

    for (int i = 0; i < n; i++) {
      for (int j = 0; j <= i; j++) {
        double sum = 0.0;
        // Compute the sum of squares for diagonal elements
        if (j == i) {
          for (int k = 0; k < j; k++) {
            sum += L(j, k) * L(j, k);
          }

          L(j, j) = std::sqrt(m_matrix[j][j] - sum);
        }
        // Compute off-diagonal elements
        else {
          for (int k = 0; k < j; k++) {
            sum += L(i, k) * L(j, k);
        }

        L(i, j) = (m_matrix[i][j] - sum) / L(j, j);
        }
      }
    }
    return L;
  }

    double mean(void) {
      double sum = 0;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          sum += this-> m_matrix[i][j];
        }
      }
      return sum / (rowSize * colSize);
    }

    double trace(void) {
      double trace = 0;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          if (i == j) {
            trace += this-> m_matrix[i][j];
          }
        }
      }
      return trace;
    }

    double minCoeff(void) {
      double min = MAXFLOAT;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          if (this-> m_matrix[i][j] < min) {
            min = this-> m_matrix[i][j];
          }
        }
      }
      return min;
    }

    double maxCoeff(void) {
      double max = 0;
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          if (this-> m_matrix[i][j] > max) {
            max = this-> m_matrix[i][j];
          }
        }
      }
      return max;
    }

    void abs(void) {
      for (unsigned i = 0; i < rowSize; i++) {
        for (unsigned j = 0; j < colSize; j++) {
          this-> m_matrix[i][j] = fabs(this-> m_matrix[i][j]);
        }
      }
    }

    double size(void) {
      return this->m_matrix.size();
    }

    /* END OF NEW METHODS */

    Matrix sub(int startRow, int startCol) {
      Matrix Sub(rowSize - startRow, colSize - startCol);
      for (unsigned i = startRow; i < rowSize; i++) {
        for (unsigned j = startCol; j < colSize; j++) {
          Sub(i - startRow, j - startCol) = this-> m_matrix[i][j];
        }
      }
      return Sub;
    }

    static Matrix createIdentity(int rowSizes, int colSizes, double diagonal = 1, double other = 0) {
      if (rowSizes != colSizes) {
        throw std::logic_error("Matrix Dimensions Don't Match");
      }

      Matrix iden(rowSizes, colSizes);
      for (unsigned i = 0; i < rowSizes; i++) {
        for (unsigned j = 0; j < colSizes; j++) {
          if (i == j) {
            iden(i, j) = diagonal;
          } else {
            iden(i, j) = other;
          }
        }
      }
      return iden;
    }

    Matrix minor_matrix(int rowNum, int colNum) {
      if (rowNum < 0 || rowNum >= rowSize || colNum < 0 || colNum >= colSize) {
        throw std::exception();
      }
      Matrix result(rowSize - 1, colSize - 1);
      int curi = 0, curj = 0;
      for (int i = 0; i < rowSize; i++) {
        if (i == rowNum) {
          curi--;
        }
        for (int j = 0; j < colSize; j++) {
          if (j == colNum) {
            curj--;
          }
          if (i != rowNum && j != colNum) {
            result(curi, curj) = m_matrix[i][j];
          }
          curj++;
        }
        curj = 0;
        curi++;
      }
      return result;
    }

    Matrix transpose() {
      Matrix Transpose(colSize, rowSize);
      for (unsigned i = 0; i < colSize; i++) {
        for (unsigned j = 0; j < rowSize; j++) {
          Transpose(i,j) = this-> m_matrix[j][i];
        }
      }
      return Transpose;
    }

    double determinant() {
      if (rowSize != colSize) {
        throw std::logic_error("Non-Square Matrix, Determinant Cannot be Found");
      }
      if (rowSize == 1 && colSize == 1) {
        return m_matrix[0][0];
      } else if (rowSize == 2 && colSize == 2) {
        return m_matrix[0][0] * m_matrix[1][1] - m_matrix[0][1] * m_matrix[1][0];
      } else {
        double Determinant = 0; int sign = 0;
        for (int i = 0; i < colSize; ++i) {
          Matrix sub(rowSize - 1, colSize - 1);
          for (int j = 1; j < rowSize; ++j) {
            for (int k = 0, l = 0; k < colSize; ++k) {
              if (k == i) {
                continue;
              }
              sub(j-1, l++) = m_matrix[j][k];
            }
          }
          sign = (i % 2 == 0) ? 1 : -1;
          Determinant += sign * m_matrix[0][i] * sub.determinant();
        }
        return Determinant;
      }
    }

    Matrix inverse() {
      double det = determinant();
      // if (det == 0) {
      //   throw std::logic_error("Determinant == 0, Matrix Inverse Cannot be Found");
      // }

      Matrix Inverse(rowSize, colSize);
      for (int i = 0; i < rowSize; i++) {
        for (int j = 0; j < colSize; j++) {
          Inverse(i, j) = (((j + i) % 2 == 0) ? 1 : -1) * minor_matrix(j, i).determinant();
        }
		  }
      Inverse /= det;
      return Inverse;
    }

    // Power Iteration
    tuple<Matrix, double, int> powerIter(unsigned rowNum, double tolerance) {
      // Picks a classic X vector
      Matrix X(rowNum, 1, 1.0);
      // Initiates X vector with values 1,2,3,4
      for (unsigned i = 1; i <= rowNum; i++) {
        X(i - 1, 0) = i;
      }

      int errorCode = 0;
      double difference = 1.0;  // Initiall value greater than tolerance
      unsigned j = 0;
      unsigned location;
      // Defined to find the value between last two eigen values
      vector<double> eigen;
      double eigenvalue = 0.0;
      eigen.push_back(0.0);

      while (fabs(difference) > tolerance) {  // breaks out when reached tolerance
        j++;
        // Normalize X vector with infinite norm
        for (int i = 0; i < rowNum; ++i) {
          eigenvalue = X(0, 0);
          if (fabs(X(i, 0)) >= fabs(eigenvalue)) {
            // Take the value of the infinite norm as your eigenvalue
            eigenvalue = X(i, 0);
            location = i;
          }
        }

        if (j >= 5e5) {
          std::cout << "Oops, that was a nasty complex number wasn't it?\n";
          std::cout << "ERROR! Returning code black, code black!";
          errorCode = -1;
          return std::make_tuple(X, 0.0, errorCode);
        }
        
        eigen.push_back(eigenvalue);
        difference = eigen[j] - eigen[j - 1];
        // Normalize X vector with its infinite norm
        X = X / eigenvalue;

        // Multiply The matrix with X vector
        X = (*this) * X;
      }

      // Take the X vector and what you've found is an eigenvector!
      X = X / eigenvalue;

      X.PrintMat();

      cout << "EigenValue" << eigenvalue << endl;

      return std::make_tuple(X, eigenvalue, errorCode);
    }
    
    // Deflation
    Matrix deflation(Matrix &X, double &eigenvalue) {
      // Deflation formula exactly applied
      double denominator = eigenvalue / (X.transpose() * X)(0, 0);
      Matrix Xtrans = X.transpose();
      Matrix RHS = (X * Xtrans);
      Matrix RHS2 = RHS * denominator;
      Matrix A2 = *this - RHS2;
      return A2;
    }
    
  };