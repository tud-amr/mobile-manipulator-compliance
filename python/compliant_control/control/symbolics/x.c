/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) x_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};

/* x:(i0[6])->(o0[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a5, a6, a7, a8, a9;
  a0=2.8500000000000001e-02;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a1=sin(a1);
  a6=-3.6732051036381108e-06;
  a3=sin(a3);
  a7=(a6*a3);
  a8=(a1*a7);
  a5=(a5-a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=cos(a8);
  a10=(a5*a9);
  a11=(a2*a3);
  a12=(a6*a4);
  a13=(a1*a12);
  a11=(a11+a13);
  a13=-9.9999999997301536e-01;
  a8=sin(a8);
  a14=(a13*a8);
  a15=(a11*a14);
  a16=-9.9999999999325395e-01;
  a17=(a16*a1);
  a18=7.3464102066435888e-06;
  a19=(a18*a8);
  a20=(a17*a19);
  a15=(a15+a20);
  a10=(a10-a15);
  a15=arg[0]? arg[0][3] : 0;
  a20=cos(a15);
  a21=(a10*a20);
  a5=(a5*a8);
  a22=(a13*a9);
  a23=(a11*a22);
  a18=(a18*a9);
  a24=(a17*a18);
  a23=(a23+a24);
  a5=(a5+a23);
  a15=sin(a15);
  a23=(a6*a15);
  a24=(a5*a23);
  a25=-7.3464102066435888e-06;
  a26=(a25*a11);
  a17=(a13*a17);
  a26=(a26+a17);
  a17=9.9999999999325395e-01;
  a27=(a17*a15);
  a28=(a26*a27);
  a24=(a24+a28);
  a21=(a21-a24);
  a24=(a0*a21);
  a28=1.0500000000000000e-01;
  a29=(a16*a5);
  a30=(a6*a26);
  a29=(a29+a30);
  a30=(a28*a29);
  a24=(a24-a30);
  a30=-2.9999999999999999e-02;
  a31=(a30*a1);
  a32=2.8000000000000003e-01;
  a11=(a32*a11);
  a31=(a31+a11);
  a11=-1.4000000000000001e-01;
  a33=(a11*a5);
  a34=2.0000000000000000e-02;
  a35=(a34*a26);
  a33=(a33+a35);
  a31=(a31+a33);
  a24=(a24-a31);
  a31=-1.0500000000000000e-01;
  a33=arg[0]? arg[0][4] : 0;
  a35=cos(a33);
  a36=(a6*a35);
  a37=(a21*a36);
  a10=(a10*a15);
  a38=(a6*a20);
  a5=(a5*a38);
  a39=(a17*a20);
  a26=(a26*a39);
  a5=(a5+a26);
  a10=(a10+a5);
  a33=sin(a33);
  a10=(a10*a33);
  a35=(a16*a35);
  a5=(a29*a35);
  a10=(a10+a5);
  a37=(a37-a10);
  a10=(a31*a37);
  a21=(a17*a21);
  a29=(a6*a29);
  a21=(a21-a29);
  a29=(a0*a21);
  a10=(a10+a29);
  a24=(a24+a10);
  a10=8.9999999999999997e-02;
  a37=(a16*a37);
  a21=(a6*a21);
  a37=(a37+a21);
  a37=(a10*a37);
  a24=(a24+a37);
  if (res[0]!=0) res[0][0]=a24;
  a30=(a30*a2);
  a12=(a2*a12);
  a24=(a1*a3);
  a12=(a12-a24);
  a24=(a32*a12);
  a30=(a30+a24);
  a24=(a12*a22);
  a37=(a16*a2);
  a21=(a37*a18);
  a24=(a24+a21);
  a1=(a1*a4);
  a2=(a2*a7);
  a1=(a1+a2);
  a2=(a1*a8);
  a24=(a24-a2);
  a2=(a11*a24);
  a7=(a25*a12);
  a13=(a13*a37);
  a7=(a7+a13);
  a13=(a34*a7);
  a2=(a2+a13);
  a30=(a30+a2);
  a1=(a1*a9);
  a12=(a12*a14);
  a37=(a37*a19);
  a12=(a12+a37);
  a1=(a1+a12);
  a12=(a1*a20);
  a37=(a24*a23);
  a2=(a7*a27);
  a37=(a37+a2);
  a12=(a12+a37);
  a37=(a0*a12);
  a2=(a16*a24);
  a13=(a6*a7);
  a2=(a2+a13);
  a13=(a28*a2);
  a37=(a37+a13);
  a30=(a30+a37);
  a37=(a12*a36);
  a24=(a24*a38);
  a7=(a7*a39);
  a24=(a24+a7);
  a1=(a1*a15);
  a24=(a24-a1);
  a24=(a24*a33);
  a1=(a2*a35);
  a24=(a24+a1);
  a37=(a37+a24);
  a24=(a31*a37);
  a12=(a17*a12);
  a2=(a6*a2);
  a12=(a12+a2);
  a2=(a0*a12);
  a24=(a24+a2);
  a30=(a30+a24);
  a37=(a16*a37);
  a12=(a6*a12);
  a37=(a37+a12);
  a37=(a10*a37);
  a30=(a30+a37);
  if (res[0]!=0) res[0][1]=a30;
  a30=2.4325000000000002e-01;
  a4=(a17*a4);
  a32=(a32*a4);
  a30=(a30+a32);
  a22=(a4*a22);
  a18=(a6*a18);
  a22=(a22+a18);
  a3=(a17*a3);
  a8=(a3*a8);
  a22=(a22-a8);
  a11=(a11*a22);
  a25=(a25*a4);
  a8=3.6732051035389906e-06;
  a25=(a25+a8);
  a34=(a34*a25);
  a11=(a11+a34);
  a30=(a30+a11);
  a3=(a3*a9);
  a4=(a4*a14);
  a19=(a6*a19);
  a4=(a4+a19);
  a3=(a3+a4);
  a20=(a3*a20);
  a23=(a22*a23);
  a27=(a25*a27);
  a23=(a23+a27);
  a20=(a20+a23);
  a23=(a0*a20);
  a27=(a16*a22);
  a4=(a6*a25);
  a27=(a27+a4);
  a28=(a28*a27);
  a23=(a23+a28);
  a30=(a30+a23);
  a36=(a20*a36);
  a22=(a22*a38);
  a25=(a25*a39);
  a22=(a22+a25);
  a3=(a3*a15);
  a22=(a22-a3);
  a22=(a22*a33);
  a35=(a27*a35);
  a22=(a22+a35);
  a36=(a36+a22);
  a31=(a31*a36);
  a17=(a17*a20);
  a27=(a6*a27);
  a17=(a17+a27);
  a0=(a0*a17);
  a31=(a31+a0);
  a30=(a30+a31);
  a16=(a16*a36);
  a6=(a6*a17);
  a16=(a16+a6);
  a10=(a10*a16);
  a30=(a30+a10);
  if (res[0]!=0) res[0][2]=a30;
  return 0;
}

CASADI_SYMBOL_EXPORT int x(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int x_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int x_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void x_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int x_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void x_release(int mem) {
}

CASADI_SYMBOL_EXPORT void x_incref(void) {
}

CASADI_SYMBOL_EXPORT void x_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int x_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int x_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real x_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* x_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* x_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* x_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* x_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int x_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif