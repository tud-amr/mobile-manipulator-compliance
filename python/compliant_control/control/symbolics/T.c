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
  #define CASADI_PREFIX(ID) T_ ## ID
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

/* T:(i0[6],i1[3])->(o0[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a5, a6, a7, a8, a9;
  a0=2.8500000000000001e-02;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a6=sin(a1);
  a7=-3.6732051036381108e-06;
  a8=sin(a3);
  a9=(a7*a8);
  a10=(a6*a9);
  a5=(a5-a10);
  a10=arg[0]? arg[0][2] : 0;
  a11=cos(a10);
  a12=(a5*a11);
  a13=(a2*a8);
  a14=(a7*a4);
  a15=(a6*a14);
  a13=(a13+a15);
  a15=-9.9999999997301536e-01;
  a16=sin(a10);
  a17=(a15*a16);
  a18=(a13*a17);
  a19=-9.9999999999325395e-01;
  a20=(a19*a6);
  a21=7.3464102066435888e-06;
  a22=(a21*a16);
  a23=(a20*a22);
  a18=(a18+a23);
  a12=(a12-a18);
  a18=arg[0]? arg[0][3] : 0;
  a23=cos(a18);
  a24=(a12*a23);
  a5=(a5*a16);
  a25=(a15*a11);
  a26=(a13*a25);
  a27=(a21*a11);
  a28=(a20*a27);
  a26=(a26+a28);
  a5=(a5+a26);
  a26=sin(a18);
  a28=(a7*a26);
  a29=(a5*a28);
  a30=-7.3464102066435888e-06;
  a31=(a30*a13);
  a20=(a15*a20);
  a31=(a31+a20);
  a20=9.9999999999325395e-01;
  a32=(a20*a26);
  a33=(a31*a32);
  a29=(a29+a33);
  a24=(a24-a29);
  a29=(a0*a24);
  a33=1.0500000000000000e-01;
  a34=(a19*a5);
  a35=(a7*a31);
  a34=(a34+a35);
  a35=(a33*a34);
  a29=(a29-a35);
  a35=-2.9999999999999999e-02;
  a36=(a35*a6);
  a37=2.8000000000000003e-01;
  a13=(a37*a13);
  a36=(a36+a13);
  a13=-1.4000000000000001e-01;
  a38=(a13*a5);
  a39=2.0000000000000000e-02;
  a40=(a39*a31);
  a38=(a38+a40);
  a36=(a36+a38);
  a29=(a29-a36);
  a36=-1.0500000000000000e-01;
  a38=arg[0]? arg[0][4] : 0;
  a40=cos(a38);
  a41=(a7*a40);
  a42=(a24*a41);
  a12=(a12*a26);
  a43=(a7*a23);
  a5=(a5*a43);
  a44=(a20*a23);
  a31=(a31*a44);
  a5=(a5+a31);
  a12=(a12+a5);
  a5=sin(a38);
  a12=(a12*a5);
  a40=(a19*a40);
  a31=(a34*a40);
  a12=(a12+a31);
  a42=(a42-a12);
  a12=(a36*a42);
  a24=(a20*a24);
  a34=(a7*a34);
  a24=(a24-a34);
  a34=(a0*a24);
  a12=(a12+a34);
  a29=(a29+a12);
  a12=1.5600000000000000e-01;
  a42=(a19*a42);
  a24=(a7*a24);
  a42=(a42+a24);
  a42=(a12*a42);
  a29=(a29+a42);
  a42=arg[1]? arg[1][1] : 0;
  a24=(a29*a42);
  a34=(a35*a2);
  a14=(a2*a14);
  a31=(a6*a8);
  a14=(a14-a31);
  a31=(a37*a14);
  a34=(a34+a31);
  a31=(a14*a25);
  a45=(a19*a2);
  a46=(a45*a27);
  a31=(a31+a46);
  a6=(a6*a4);
  a2=(a2*a9);
  a6=(a6+a2);
  a2=(a6*a16);
  a31=(a31-a2);
  a2=(a13*a31);
  a9=(a30*a14);
  a46=(a15*a45);
  a9=(a9+a46);
  a46=(a39*a9);
  a2=(a2+a46);
  a34=(a34+a2);
  a6=(a6*a11);
  a14=(a14*a17);
  a45=(a45*a22);
  a14=(a14+a45);
  a6=(a6+a14);
  a14=(a6*a23);
  a45=(a31*a28);
  a2=(a9*a32);
  a45=(a45+a2);
  a14=(a14+a45);
  a45=(a0*a14);
  a2=(a19*a31);
  a46=(a7*a9);
  a2=(a2+a46);
  a46=(a33*a2);
  a45=(a45+a46);
  a34=(a34+a45);
  a45=(a14*a41);
  a31=(a31*a43);
  a9=(a9*a44);
  a31=(a31+a9);
  a6=(a6*a26);
  a31=(a31-a6);
  a31=(a31*a5);
  a6=(a2*a40);
  a31=(a31+a6);
  a45=(a45+a31);
  a31=(a36*a45);
  a14=(a20*a14);
  a2=(a7*a2);
  a14=(a14+a2);
  a2=(a0*a14);
  a31=(a31+a2);
  a34=(a34+a31);
  a45=(a19*a45);
  a14=(a7*a14);
  a45=(a45+a14);
  a45=(a12*a45);
  a34=(a34+a45);
  a45=arg[1]? arg[1][0] : 0;
  a14=(a34*a45);
  a24=(a24-a14);
  if (res[0]!=0) res[0][0]=a24;
  a24=cos(a1);
  a14=(a35*a24);
  a31=(a7*a14);
  a2=2.4325000000000002e-01;
  a6=(a19*a24);
  a9=(a2*a6);
  a31=(a31-a9);
  a9=(a7*a34);
  a4=(a20*a4);
  a46=(a37*a4);
  a46=(a2+a46);
  a25=(a4*a25);
  a27=(a7*a27);
  a25=(a25+a27);
  a8=(a20*a8);
  a16=(a8*a16);
  a25=(a25-a16);
  a16=(a13*a25);
  a27=(a30*a4);
  a47=3.6732051035389906e-06;
  a27=(a27+a47);
  a48=(a39*a27);
  a16=(a16+a48);
  a46=(a46+a16);
  a8=(a8*a11);
  a4=(a4*a17);
  a22=(a7*a22);
  a4=(a4+a22);
  a8=(a8+a4);
  a23=(a8*a23);
  a28=(a25*a28);
  a32=(a27*a32);
  a28=(a28+a32);
  a23=(a23+a28);
  a28=(a0*a23);
  a32=(a19*a25);
  a4=(a7*a27);
  a32=(a32+a4);
  a4=(a33*a32);
  a28=(a28+a4);
  a46=(a46+a28);
  a41=(a23*a41);
  a25=(a25*a43);
  a27=(a27*a44);
  a25=(a25+a27);
  a8=(a8*a26);
  a25=(a25-a8);
  a25=(a25*a5);
  a40=(a32*a40);
  a25=(a25+a40);
  a41=(a41+a25);
  a25=(a36*a41);
  a23=(a20*a23);
  a32=(a7*a32);
  a23=(a23+a32);
  a32=(a0*a23);
  a25=(a25+a32);
  a46=(a46+a25);
  a41=(a19*a41);
  a23=(a7*a23);
  a41=(a41+a23);
  a12=(a12*a41);
  a46=(a46+a12);
  a12=(a46*a6);
  a9=(a9-a12);
  a31=(a31-a9);
  a31=(a31*a45);
  a1=sin(a1);
  a35=(a35*a1);
  a9=(a7*a35);
  a12=(a19*a1);
  a41=(a2*a12);
  a9=(a9-a41);
  a41=(a46*a12);
  a23=(a7*a29);
  a41=(a41+a23);
  a9=(a9+a41);
  a9=(a9*a42);
  a31=(a31+a9);
  a9=(a14*a12);
  a41=(a35*a6);
  a9=(a9-a41);
  a41=(a29*a6);
  a23=(a34*a12);
  a41=(a41+a23);
  a9=(a9-a41);
  a41=arg[1]? arg[1][2] : 0;
  a9=(a9*a41);
  a31=(a31+a9);
  if (res[0]!=0) res[0][1]=a31;
  a31=cos(a3);
  a9=(a7*a31);
  a23=(a24*a9);
  a3=sin(a3);
  a25=(a1*a3);
  a23=(a23-a25);
  a25=(a37*a23);
  a14=(a14+a25);
  a25=(a20*a31);
  a32=(a30*a25);
  a32=(a32+a47);
  a47=(a14*a32);
  a40=(a37*a25);
  a2=(a2+a40);
  a40=(a30*a23);
  a5=(a15*a6);
  a40=(a40+a5);
  a5=(a2*a40);
  a47=(a47-a5);
  a5=(a34*a32);
  a8=(a46*a40);
  a5=(a5-a8);
  a47=(a47-a5);
  a47=(a47*a45);
  a5=(a24*a3);
  a9=(a1*a9);
  a5=(a5+a9);
  a37=(a37*a5);
  a35=(a35+a37);
  a37=(a35*a32);
  a30=(a30*a5);
  a9=(a15*a12);
  a30=(a30+a9);
  a9=(a2*a30);
  a37=(a37-a9);
  a9=(a46*a30);
  a8=(a29*a32);
  a9=(a9+a8);
  a37=(a37+a9);
  a37=(a37*a42);
  a47=(a47+a37);
  a37=(a14*a30);
  a9=(a35*a40);
  a37=(a37-a9);
  a9=(a29*a40);
  a8=(a34*a30);
  a9=(a9+a8);
  a37=(a37-a9);
  a37=(a37*a41);
  a47=(a47+a37);
  if (res[0]!=0) res[0][2]=a47;
  a47=cos(a10);
  a37=(a15*a47);
  a9=(a23*a37);
  a8=(a21*a47);
  a26=(a6*a8);
  a9=(a9+a26);
  a26=(a1*a31);
  a27=(a7*a3);
  a44=(a24*a27);
  a26=(a26+a44);
  a10=sin(a10);
  a44=(a26*a10);
  a9=(a9-a44);
  a44=(a13*a9);
  a43=(a39*a40);
  a44=(a44+a43);
  a14=(a14+a44);
  a44=(a25*a37);
  a43=(a7*a8);
  a44=(a44+a43);
  a3=(a20*a3);
  a43=(a3*a10);
  a44=(a44-a43);
  a43=(a19*a44);
  a28=(a7*a32);
  a43=(a43+a28);
  a28=(a14*a43);
  a4=(a13*a44);
  a22=(a39*a32);
  a4=(a4+a22);
  a2=(a2+a4);
  a4=(a19*a9);
  a22=(a7*a40);
  a4=(a4+a22);
  a22=(a2*a4);
  a28=(a28-a22);
  a22=(a34*a43);
  a17=(a46*a4);
  a22=(a22-a17);
  a28=(a28-a22);
  a28=(a28*a45);
  a24=(a24*a31);
  a1=(a1*a27);
  a24=(a24-a1);
  a1=(a24*a10);
  a37=(a5*a37);
  a8=(a12*a8);
  a37=(a37+a8);
  a1=(a1+a37);
  a13=(a13*a1);
  a39=(a39*a30);
  a13=(a13+a39);
  a35=(a35+a13);
  a13=(a35*a43);
  a39=(a19*a1);
  a37=(a7*a30);
  a39=(a39+a37);
  a37=(a2*a39);
  a13=(a13-a37);
  a37=(a46*a39);
  a8=(a29*a43);
  a37=(a37+a8);
  a13=(a13+a37);
  a13=(a13*a42);
  a28=(a28+a13);
  a13=(a14*a39);
  a37=(a35*a4);
  a13=(a13-a37);
  a37=(a29*a4);
  a8=(a34*a39);
  a37=(a37+a8);
  a13=(a13-a37);
  a13=(a13*a41);
  a28=(a28+a13);
  if (res[0]!=0) res[0][3]=a28;
  a26=(a26*a47);
  a15=(a15*a10);
  a23=(a23*a15);
  a21=(a21*a10);
  a6=(a6*a21);
  a23=(a23+a6);
  a26=(a26+a23);
  a23=cos(a18);
  a6=(a26*a23);
  a18=sin(a18);
  a10=(a7*a18);
  a28=(a9*a10);
  a13=(a20*a18);
  a37=(a40*a13);
  a28=(a28+a37);
  a6=(a6+a28);
  a28=(a0*a6);
  a37=(a33*a4);
  a28=(a28+a37);
  a14=(a14+a28);
  a3=(a3*a47);
  a25=(a25*a15);
  a28=(a7*a21);
  a25=(a25+a28);
  a3=(a3+a25);
  a25=(a3*a23);
  a28=(a44*a10);
  a37=(a32*a13);
  a28=(a28+a37);
  a25=(a25+a28);
  a28=(a20*a25);
  a37=(a7*a43);
  a28=(a28+a37);
  a37=(a14*a28);
  a8=(a0*a25);
  a27=(a33*a43);
  a8=(a8+a27);
  a2=(a2+a8);
  a8=(a20*a6);
  a27=(a7*a4);
  a8=(a8+a27);
  a27=(a2*a8);
  a37=(a37-a27);
  a27=(a34*a28);
  a31=(a46*a8);
  a27=(a27-a31);
  a37=(a37-a27);
  a37=(a37*a45);
  a24=(a24*a47);
  a5=(a5*a15);
  a12=(a12*a21);
  a5=(a5+a12);
  a24=(a24-a5);
  a5=(a24*a23);
  a10=(a1*a10);
  a13=(a30*a13);
  a10=(a10+a13);
  a5=(a5-a10);
  a10=(a20*a5);
  a13=(a7*a39);
  a10=(a10-a13);
  a13=(a2*a10);
  a12=(a0*a5);
  a33=(a33*a39);
  a12=(a12-a33);
  a12=(a12-a35);
  a35=(a12*a28);
  a13=(a13-a35);
  a35=(a46*a10);
  a33=(a29*a28);
  a35=(a35-a33);
  a13=(a13-a35);
  a13=(a13*a42);
  a37=(a37+a13);
  a13=(a12*a8);
  a35=(a14*a10);
  a13=(a13-a35);
  a35=(a29*a8);
  a33=(a34*a10);
  a35=(a35-a33);
  a13=(a13-a35);
  a13=(a13*a41);
  a37=(a37+a13);
  if (res[0]!=0) res[0][4]=a37;
  a37=cos(a38);
  a13=(a7*a37);
  a6=(a6*a13);
  a35=(a7*a23);
  a9=(a9*a35);
  a20=(a20*a23);
  a40=(a40*a20);
  a9=(a9+a40);
  a26=(a26*a18);
  a9=(a9-a26);
  a38=sin(a38);
  a9=(a9*a38);
  a37=(a19*a37);
  a4=(a4*a37);
  a9=(a9+a4);
  a6=(a6+a9);
  a9=(a36*a6);
  a4=(a0*a8);
  a9=(a9+a4);
  a14=(a14+a9);
  a25=(a25*a13);
  a44=(a44*a35);
  a32=(a32*a20);
  a44=(a44+a32);
  a3=(a3*a18);
  a44=(a44-a3);
  a44=(a44*a38);
  a43=(a43*a37);
  a44=(a44+a43);
  a25=(a25+a44);
  a44=(a19*a25);
  a43=(a7*a28);
  a44=(a44+a43);
  a43=(a14*a44);
  a25=(a36*a25);
  a28=(a0*a28);
  a25=(a25+a28);
  a2=(a2+a25);
  a6=(a19*a6);
  a8=(a7*a8);
  a6=(a6+a8);
  a8=(a2*a6);
  a43=(a43-a8);
  a8=(a34*a44);
  a25=(a46*a6);
  a8=(a8-a25);
  a43=(a43-a8);
  a43=(a43*a45);
  a5=(a5*a13);
  a24=(a24*a18);
  a1=(a1*a35);
  a30=(a30*a20);
  a1=(a1+a30);
  a24=(a24+a1);
  a24=(a24*a38);
  a39=(a39*a37);
  a24=(a24+a39);
  a5=(a5-a24);
  a19=(a19*a5);
  a7=(a7*a10);
  a19=(a19+a7);
  a2=(a2*a19);
  a36=(a36*a5);
  a0=(a0*a10);
  a36=(a36+a0);
  a12=(a12+a36);
  a36=(a12*a44);
  a2=(a2-a36);
  a46=(a46*a19);
  a44=(a29*a44);
  a46=(a46-a44);
  a2=(a2-a46);
  a2=(a2*a42);
  a43=(a43+a2);
  a12=(a12*a6);
  a14=(a14*a19);
  a12=(a12-a14);
  a29=(a29*a6);
  a34=(a34*a19);
  a29=(a29-a34);
  a12=(a12-a29);
  a12=(a12*a41);
  a43=(a43+a12);
  if (res[0]!=0) res[0][5]=a43;
  return 0;
}

CASADI_SYMBOL_EXPORT int T(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int T_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int T_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void T_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int T_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void T_release(int mem) {
}

CASADI_SYMBOL_EXPORT void T_incref(void) {
}

CASADI_SYMBOL_EXPORT void T_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int T_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int T_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real T_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* T_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* T_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* T_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* T_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int T_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
