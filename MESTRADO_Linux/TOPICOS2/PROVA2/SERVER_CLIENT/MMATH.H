/******************************************************************************

			MATRIX MATH TEMPLATES
	Copyright (C) 1995..2003 Walter Fetter Lages <w.fetter@ieee.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    You can also obtain a copy of the GNU General Public License
    at <http://www.gnu.org/licenses>.

*******************************************************************************/


#ifndef MMATH_H
#define MMATH_H

#define DEBUG 1


#ifndef __cplusplus

#error MMATH.H must be compiled with C++.

#endif


#include <iostream.h>
#include <math.h>
#include <stdlib.h>


/*******************************************************************************
			MATRIX template
*******************************************************************************/
template<class T> class CVECTOR;


template<class T> class MATRIX
{
	protected:

	T *ptr;
	int nlin;
	int ncol;


	public:

	MATRIX(int lin,int col);
	MATRIX(const MATRIX<T> &m);
	~MATRIX(void) { delete[] ptr; };

	int lin(void) const { return nlin; };
	int col(void) const { return ncol; };

	void eye(void);
	void zeros(void);
	void ones(void);

	MATRIX<T> t(void) const;

	MATRIX<T> inv(void) const;

	MATRIX<T> stack(const MATRIX<T> &m) const;

	MATRIX<T> operator=(const MATRIX<T> &m);

	MATRIX<T> operator+=(const MATRIX<T> &m);
	MATRIX<T> operator-=(const MATRIX<T> &m);
	MATRIX<T> operator*=(const MATRIX<T> &m);
	MATRIX<T> operator/=(const MATRIX<T> &m);


	MATRIX<T> operator+(void) const { return *this; };
	MATRIX<T> operator+(const MATRIX<T> &m) const;

	MATRIX<T> operator-(void) const;
	MATRIX<T> operator-(const MATRIX<T> &m) const;

	MATRIX<T> operator*(const MATRIX<T> &m) const;
	MATRIX<T> operator*(const T &esc) const;
//	friend MATRIX<T> operator*<>(const T &esc,const MATRIX<T> &m);

	MATRIX<T> operator/(const MATRIX<T> &m) const;
	MATRIX<T> operator/(const T &esc) const;


	operator T(void) const;

	T *operator[](int lin) const { return (ptr+lin*ncol); };

	MATRIX<T> operator()(int lin0,int lin1,int col0,int col1) const;

	friend ostream &operator<< <>(ostream &s,const MATRIX<T> &m);

};

template<class T> MATRIX<T>::MATRIX(int lin,int col)
{

#ifdef DEBUG

	if(((unsigned long)lin*(unsigned long)col) > 65535L)
	{
		cout << "\nCan't create a ";
		cout << "MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif


	ptr=new T[(nlin=lin)*(ncol=col)];

#ifdef DEBUG

	if(ptr==NULL)
	{
		cout << "\nNot enough memory to allocate ";
		cout << "MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}
#endif

}


template<class T> MATRIX<T>::MATRIX(const MATRIX<T> &m)
{

	ptr=new T[(nlin=m.lin())*(ncol=m.col())];

#ifdef DEBUG

	if(ptr==NULL)
	{
		cout << "\nNot enough memory to allocate ";
		cout << "MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}
#endif

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(ptr+i*ncol+j)=*(m.ptr+i*ncol+j);

}


template<class T> void MATRIX<T>::eye(void)
{

#ifdef DEBUG

	if(nlin != ncol)
	{
		cout << "\nCan't generate an eye MATRIX: ";
		cout << "the number of lines (" << nlin << ") is different from ";
		cout << "the number of columns (" << ncol << ").\n";
		exit (-1);
	}

#endif

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(ptr+i*ncol+j)=(i==j)? 1.0:0.0;
}

template<class T> void MATRIX<T>::zeros(void)
{
	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) (*this)[i][j]=0;
}


template<class T> void MATRIX<T>::ones(void)
{
	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) (*this)[i][j]=1;
}

template<class T> MATRIX<T> MATRIX<T>::t(void) const
{
	MATRIX<T> tp(ncol,nlin);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) tp[j][i]=(*this)[i][j];
	return tp;
}


template<class T> MATRIX<T> MATRIX<T>::inv(void) const
{

#ifdef DEBUG

	if(nlin != ncol)
	{
		cout << "\nCan't invert a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	const T tiny=1.0e-20;
	MATRIX<T> y(nlin,ncol);	

// matrix 2x2 is a special case for performance reasons
	if(nlin==2)
	{
		T det=(*this)[0][0]*(*this)[1][1]-(*this)[0][1]*(*this)[1][0];

#ifdef DEBUG
		if(fabs(det) < tiny)
		{
			cout <<"\nCan't invert a singular MATRIX(" << nlin << "," << ncol << ")\n";
			exit(-1);
		}
#endif		

		y[0][0]=(*this)[1][1]/det;
		y[0][1]=-(*this)[0][1]/det;
		y[1][0]=-(*this)[1][0]/det;
		y[1][1]=(*this)[0][0]/det;
	
		return y;
	}

// matrix invertion via LU decomposition

	MATRIX<T> a=*this;      // destructive method

	CVECTOR<T> vv(nlin);
	int d=1;
	T big;
	T sum;
	int *indx=new int[nlin];

#ifdef DEBUG

	if(indx == NULL)
	{
		cout << "\nNot enough memory to invert a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	for(int i=0;i < nlin;i++)
	{
		big=0.0;
		T temp;
		for(int j=0;j < ncol;j++) if((temp=fabs(a[i][j])) > big) big=temp;

#ifdef DEBUG

		if(big==0.0)
		{
			cout <<"\nCan't invert a singular MATRIX\n";
			exit(-1);
		}

#endif

		vv[i]=1.0/big;
	}
	for(int j=0;j < nlin;j++)
	{
		T dum;
		int imax=0;

		for(int i=0;i < j;i++)
		{
			sum=a[i][j];
			for(int k=0;k < i;k++) sum-=a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for(int i=j;i < nlin;i++)
		{
			sum=a[i][j];
			for(int k=0;k < j;k++) sum-=a[i][k]*a[k][j];
			a[i][j]=sum;
			if((dum=vv[i]*fabs(sum)) >= big)
			{
				big=dum;
				imax=i;
			}
		}
		if(j != imax)
		{
			for(int k=0;k < nlin;k++)
			{
				// interchange rows

				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			d=-d;
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if(a[j][j]==0.0) a[j][j]=tiny;
		if(j != nlin-1)
		{
			dum=1.0/a[j][j];
			for(int i=j+1;i < nlin;i++) a[i][j]*=dum;
		}
	}


	// find inverse

	int ip;

	for(int j=0;j < nlin;j++)
	{

		for(int i=0;i < nlin;i++) y[i][j]=0.0;
		y[j][j]=1.0;

		int ii=-1;
		for(int i=0;i < nlin;i++)
		{
			ip=indx[i];
			sum=y[ip][j];
			y[ip][j]=y[i][j];
			if(ii >= 0) for(int k=ii;k <= i-1;k++) sum-=a[i][k]*y[k][j];
			else if(sum) ii=i;
			y[i][j]=sum;
		}
		for(int i=nlin-1; i >= 0;i--)
		{
			sum=y[i][j];
			for(int k=i+1;k < nlin;k++) sum-=a[i][k]*y[k][j];
			y[i][j]=sum/a[i][i];
		}
	}
	delete[] indx;

	return y;
}


template<class T> inline MATRIX<T> inv(const MATRIX<T> &m)
{
	return m.inv();
}


template<class T> MATRIX<T> MATRIX<T>::stack(const MATRIX<T> &m) const
{

#ifdef DEBUG

	if((ncol != m.ncol))
	{
		cout << "\nCan't stack a MATRIX(" << m.nlin << "," << m.ncol << ")";
		cout << " onto a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	MATRIX<T> s(nlin+m.nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) s[i][j]=(*this)[i][j];
	for(int i=0;i < m.nlin;i++) for(int j=0;j < m.ncol;j++) s[i+nlin][j]=m[i][j];

	return s;

}


template<class T> MATRIX<T> MATRIX<T>::operator=(const MATRIX<T> &m)
{

#ifdef DEBUG

	if((nlin != m.nlin) || (ncol != m.ncol))
	{
		cout << "\nCan't copy a MATRIX(" << m.nlin << "," << m.ncol << ")";
		cout << " onto a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	if(this != &m)
		for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(ptr+i*ncol+j)=*(m.ptr+i*ncol+j);

	return *this;
}

template<class T> MATRIX<T> MATRIX<T>::operator+=(const MATRIX<T> &m)
{
	*this=*this+m;
	return *this;
};

template<class T> MATRIX<T> MATRIX<T>::operator-=(const MATRIX<T> &m)
{
	*this=*this-m;
	return *this;
};

template<class T> MATRIX<T> MATRIX<T>::operator*=(const MATRIX<T> &m)
{
	*this=*this*m;
	return *this;
};


template<class T> MATRIX<T> MATRIX<T>::operator/=(const MATRIX<T> &m)
{
	*this=*this*m.inv();
	return *this;
};

template<class T> MATRIX<T> MATRIX<T>::operator+(const MATRIX<T> &m) const
{

#ifdef DEBUG

	if((nlin != m.nlin) || (ncol != m.ncol))
	{
		cout << "\nCan't add a MATRIX(" << m.nlin << "," << m.ncol << ")";
		cout << " to a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	MATRIX<T> sum(nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(sum.ptr+i*ncol+j)=*(ptr+i*ncol+j) + *(m.ptr+i*ncol+j);

	return sum;
}


template<class T> MATRIX<T> MATRIX<T>::operator-(void) const
{
	MATRIX<T> s(nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(s.ptr+i*ncol+j)=-(*(ptr+i*ncol+j));
	return s;
}


template<class T> MATRIX<T> MATRIX<T>::operator-(const MATRIX<T> &m) const
{

#ifdef DEBUG

	if((nlin != m.nlin) || (ncol != m.ncol))
	{
		cout << "\nCan't subtract a MATRIX(" << m.nlin << "," << m.ncol << ")";
		cout << " from a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	MATRIX<T> sub(nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++) *(sub.ptr+i*ncol+j)=*(ptr+i*ncol+j) - *(m.ptr+i*ncol+j);

	return sub;
}

template<class T> MATRIX<T> MATRIX<T>::operator*(const MATRIX<T> &m) const
{

#ifdef DEBUG

	if(ncol != m.nlin)
	{
		cout << "\nCan't multiply a MATRIX(" << m.nlin << "," << m.ncol << ")";
		cout << " by a MATRIX(" << nlin << "," << ncol << ").\n";
		exit(-1);
	}

#endif

	MATRIX<T> prod(nlin,m.ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < m.ncol;j++)
	{
		*(prod.ptr+i*prod.ncol+j)=0.0;
		for(int k=0;k < ncol;k++) *(prod.ptr+i*prod.ncol+j)+=(*(ptr+i*ncol+k))*(*(m.ptr+k*m.ncol+j));
	}

	return prod;
}



template<class T> MATRIX<T> MATRIX<T>::operator/(const MATRIX<T> &m) const
{
	return *this*m.inv();
}



template<class T> MATRIX<T> MATRIX<T>::operator*(const T &esc) const
{

	MATRIX<T> prod(nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++)
	{
		*(prod.ptr+i*prod.ncol+j)=*(ptr+i*ncol+j)*esc;
	}

	return prod;
}

template<class T> MATRIX<T> MATRIX<T>::operator/(const T &esc) const
{

	MATRIX<T> prod(nlin,ncol);

	for(int i=0;i < nlin;i++) for(int j=0;j < ncol;j++)
	{
		*(prod.ptr+i*prod.ncol+j)=*(ptr+i*ncol+j)/esc;
	}

	return prod;
}


template<class T> MATRIX<T> operator*(const T &esc,const MATRIX<T> &m)
{

	MATRIX<T> prod(m.lin(),m.col());

	for(int i=0;i < m.lin();i++) for(int j=0;j < m.col();j++)
	{
		prod[i][j]=m[i][j]*esc;
	}

	return prod;
}


template<class T> MATRIX<T>::operator T(void) const
{

#ifdef DEBUG

	if((nlin != 1) || (ncol != 1))
	{
		cout << "\nCan't convert a MATRIX(" << nlin << "," << ncol << ")";
		cout << " to an scalar.\n";
		exit(-1);
	}

#endif
	return *ptr;
}


template<class T> MATRIX<T> MATRIX<T>::operator()(int lin0,int lin1,int col0,int col1) const
{

	int linc=(lin0 <= lin1)? 1:-1;
	int cinc=(col0 <= col1)? 1:-1;

	MATRIX<T> sm(abs(lin1-lin0)+1,abs(col1-col0)+1);

	for(int i=lin0;linc*(lin1-i) >= 0;i+=linc)
		for(int j=col0;cinc*(col1-j) >= 0;j+=cinc)
			sm[i-lin0][j-col0]=(*this)[i][j];
	return sm;
}

template<class T> ostream &operator<<(ostream &s,const MATRIX<T> &m)
{
	for(int i=0;i < m.lin();i++)
	{
		for(int j=0;j < m.col();j++) s << m[i][j] << " ";
		s << "\n";
	}
	return s;
}




/*******************************************************************************
			CVECTOR template
*******************************************************************************/

template<class T> class CVECTOR:public MATRIX<T>
{

	public:

	CVECTOR(int lin):MATRIX<T>(lin,1) { };
	CVECTOR(const CVECTOR<T> &cv);
	CVECTOR(const MATRIX<T> &m);

	CVECTOR<T> operator=(const MATRIX<T> &m);
	T &operator[](int lin) const { return *(ptr+lin); };
	CVECTOR<T> operator()(int lin0,int lin1) const;
};


template<class T> CVECTOR<T>::CVECTOR(const CVECTOR<T> &cv):MATRIX<T>(cv.lin(),1)
{
	for(int i=0;i < nlin;i++) *(ptr+i)=cv[i];

}


template<class T> CVECTOR<T>::CVECTOR(const MATRIX<T> &m):MATRIX<T>(m.lin(),1)
{

#ifdef DEBUG

	if(m.col() != 1)
	{
		cout << "\nCan't convert a MATRIX(" << m.lin() << "," << m.col() << ")";
		cout << " to a CVECTOR(" << nlin << ").\n";
		exit(-1);
	}

#endif

	if(this != &m)
		for(int i=0;i < nlin;i++) *(ptr+i)=m[i][0];
}


template<class T> CVECTOR<T> CVECTOR<T>::operator=(const MATRIX<T> &m)
{

#ifdef DEBUG

	if((m.lin() != nlin) || (m.col() != 1))
	{
		cout << "\nCan't convert a MATRIX(" << m.lin() << "," << m.col() << ")";
		cout << " to a CVECTOR(" << nlin << ").\n";
		exit(-1);
	}

#endif

	if(this != &m)
		for(int i=0;i < nlin;i++) *(ptr+i)=m[i][0];

	return *this;
}


template<class T> CVECTOR<T> CVECTOR<T>::operator()(int lin0,int lin1) const
{

	int linc=(lin0 <= lin1)? 1:-1;

	CVECTOR<T> sv(abs(lin1-lin0)+1);

	for(int i=lin0;linc*(lin1-i) >= 0;i+=linc) sv[i-lin0]=(*this)[i];

	return sv;
}


// Runge-Kutta
template<class T> CVECTOR<T> rk(CVECTOR<T> &x,CVECTOR<T> &u,const T h,CVECTOR<T> (*f)(const CVECTOR<T> &x,const CVECTOR<T> &u))
{
	const T hh=h/2.0;
	const T h6=h/6.0;


	CVECTOR<T> dx=f(x,u);

	CVECTOR<T> xt=x+hh*dx;

	CVECTOR<T> dxt=f(xt,u);

	xt=x+hh*dxt;

	CVECTOR<T> dxm=f(xt,u);

	xt=x+h*dxm;

	dxm=dxm+dxt;

	dxt=f(xt,u);

	CVECTOR<T> xout=x+h6*(dx+dxt+2.0*dxm);

	return xout;
}



/*******************************************************************************
			 LVECTOR template
*******************************************************************************/

template<class T> class LVECTOR:public MATRIX<T>
{

	public:

	LVECTOR(int col):MATRIX<T>(1,col){ };
	LVECTOR(const LVECTOR &lv);

	LVECTOR<T> operator=(const MATRIX<double> &m);
	T &operator[](int col) const { return *(ptr+col); };
	LVECTOR<T> operator()(int col0,int col1) const;
};


template<class T> LVECTOR<T>::LVECTOR(const LVECTOR<T> &lv):MATRIX<T>(1,lv.col())
{
	for(int i=0;i < ncol;i++) *(ptr+i)=lv[i];

}


template<class T> LVECTOR<T> LVECTOR<T>::operator=(const MATRIX<double> &m)
{

#ifdef DEBUG

	if((m.lin() != 1) || (m.col() != ncol))
	{
		cout << "\nCan't convert a MATRIX(" << m.lin() << "," << m.col() << ")";
		cout << " to an LVECTOR(" << ncol << ").\n";
		exit(-1);
	}

#endif

	if(this != &m)
		for(int i=0;i < ncol;i++) *(ptr+i)=m[0][i];

	return *this;
}


template<class T> LVECTOR<T> LVECTOR<T>::operator()(int col0,int col1) const
{

	int cinc=(col0 <= col1)? 1:-1;

	LVECTOR<T> sv(abs(col1-col0)+1);

	for(int j=col0;cinc*(col1-j) >= 0;j+=cinc) sv[j-col0]=(*this)[j];

	return sv;
}

#endif





