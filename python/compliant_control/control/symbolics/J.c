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
  #define CASADI_PREFIX(ID) J_ ## ID
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
static const casadi_int casadi_s1[27] = {3, 6, 0, 3, 6, 9, 12, 15, 18, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* J:(i0[6])->(o0[3x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a5, a6, a7, a8, a9;
  a0=-2.9999999999999999e-02;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=(a0*a2);
  a4=2.8000000000000003e-01;
  a5=-3.6732051036381108e-06;
  a6=arg[0]? arg[0][1] : 0;
  a7=cos(a6);
  a8=(a5*a7);
  a9=(a2*a8);
  a10=sin(a1);
  a11=sin(a6);
  a12=(a10*a11);
  a9=(a9-a12);
  a12=(a4*a9);
  a3=(a3+a12);
  a12=-1.4000000000000001e-01;
  a13=-9.9999999997301536e-01;
  a14=arg[0]? arg[0][2] : 0;
  a15=cos(a14);
  a16=(a13*a15);
  a17=(a9*a16);
  a18=-9.9999999999325395e-01;
  a19=(a18*a2);
  a20=7.3464102066435888e-06;
  a21=(a20*a15);
  a22=(a19*a21);
  a17=(a17+a22);
  a22=(a10*a7);
  a23=(a5*a11);
  a24=(a2*a23);
  a22=(a22+a24);
  a24=sin(a14);
  a25=(a22*a24);
  a17=(a17-a25);
  a25=(a12*a17);
  a26=2.0000000000000000e-02;
  a27=-7.3464102066435888e-06;
  a28=(a27*a9);
  a29=(a13*a19);
  a28=(a28+a29);
  a29=(a26*a28);
  a25=(a25+a29);
  a3=(a3+a25);
  a25=2.8500000000000001e-02;
  a22=(a22*a15);
  a29=(a13*a24);
  a9=(a9*a29);
  a30=(a20*a24);
  a19=(a19*a30);
  a9=(a9+a19);
  a22=(a22+a9);
  a9=arg[0]? arg[0][3] : 0;
  a19=cos(a9);
  a31=(a22*a19);
  a32=sin(a9);
  a33=(a5*a32);
  a34=(a17*a33);
  a35=9.9999999999325395e-01;
  a36=(a35*a32);
  a37=(a28*a36);
  a34=(a34+a37);
  a31=(a31+a34);
  a34=(a25*a31);
  a37=1.0500000000000000e-01;
  a38=(a18*a17);
  a39=(a5*a28);
  a38=(a38+a39);
  a39=(a37*a38);
  a34=(a34+a39);
  a3=(a3+a34);
  a34=-1.0500000000000000e-01;
  a39=arg[0]? arg[0][4] : 0;
  a40=cos(a39);
  a41=(a5*a40);
  a42=(a31*a41);
  a43=(a5*a19);
  a17=(a17*a43);
  a44=(a35*a19);
  a28=(a28*a44);
  a17=(a17+a28);
  a22=(a22*a32);
  a17=(a17-a22);
  a22=sin(a39);
  a17=(a17*a22);
  a40=(a18*a40);
  a28=(a38*a40);
  a17=(a17+a28);
  a42=(a42+a17);
  a17=(a34*a42);
  a31=(a35*a31);
  a38=(a5*a38);
  a31=(a31+a38);
  a38=(a25*a31);
  a17=(a17+a38);
  a3=(a3+a17);
  a17=8.9999999999999997e-02;
  a42=(a18*a42);
  a31=(a5*a31);
  a42=(a42+a31);
  a42=(a17*a42);
  a3=(a3+a42);
  a42=(-a3);
  if (res[0]!=0) res[0][0]=a42;
  a42=(a2*a7);
  a23=(a10*a23);
  a42=(a42-a23);
  a23=(a42*a15);
  a2=(a2*a11);
  a8=(a10*a8);
  a2=(a2+a8);
  a8=(a2*a29);
  a31=(a18*a10);
  a38=(a31*a30);
  a8=(a8+a38);
  a23=(a23-a8);
  a8=(a23*a19);
  a42=(a42*a24);
  a38=(a2*a16);
  a28=(a31*a21);
  a38=(a38+a28);
  a42=(a42+a38);
  a38=(a42*a33);
  a28=(a27*a2);
  a31=(a13*a31);
  a28=(a28+a31);
  a31=(a28*a36);
  a38=(a38+a31);
  a8=(a8-a38);
  a38=(a25*a8);
  a31=(a18*a42);
  a45=(a5*a28);
  a31=(a31+a45);
  a45=(a37*a31);
  a38=(a38-a45);
  a10=(a0*a10);
  a2=(a4*a2);
  a10=(a10+a2);
  a2=(a12*a42);
  a45=(a26*a28);
  a2=(a2+a45);
  a10=(a10+a2);
  a38=(a38-a10);
  a10=(a8*a41);
  a23=(a23*a32);
  a42=(a42*a43);
  a28=(a28*a44);
  a42=(a42+a28);
  a23=(a23+a42);
  a23=(a23*a22);
  a42=(a31*a40);
  a23=(a23+a42);
  a10=(a10-a23);
  a23=(a34*a10);
  a8=(a35*a8);
  a31=(a5*a31);
  a8=(a8-a31);
  a31=(a25*a8);
  a23=(a23+a31);
  a38=(a38+a23);
  a10=(a18*a10);
  a8=(a5*a8);
  a10=(a10+a8);
  a10=(a17*a10);
  a38=(a38+a10);
  if (res[0]!=0) res[0][1]=a38;
  a10=0.;
  if (res[0]!=0) res[0][2]=a10;
  a10=cos(a1);
  a8=(a0*a10);
  a23=(a5*a8);
  a31=2.4325000000000002e-01;
  a42=(a18*a10);
  a28=(a31*a42);
  a23=(a23-a28);
  a28=(a5*a3);
  a7=(a35*a7);
  a2=(a4*a7);
  a2=(a31+a2);
  a16=(a7*a16);
  a21=(a5*a21);
  a16=(a16+a21);
  a11=(a35*a11);
  a24=(a11*a24);
  a16=(a16-a24);
  a24=(a12*a16);
  a21=(a27*a7);
  a45=3.6732051035389906e-06;
  a21=(a21+a45);
  a46=(a26*a21);
  a24=(a24+a46);
  a2=(a2+a24);
  a11=(a11*a15);
  a7=(a7*a29);
  a30=(a5*a30);
  a7=(a7+a30);
  a11=(a11+a7);
  a19=(a11*a19);
  a33=(a16*a33);
  a36=(a21*a36);
  a33=(a33+a36);
  a19=(a19+a33);
  a33=(a25*a19);
  a36=(a18*a16);
  a7=(a5*a21);
  a36=(a36+a7);
  a7=(a37*a36);
  a33=(a33+a7);
  a2=(a2+a33);
  a41=(a19*a41);
  a16=(a16*a43);
  a21=(a21*a44);
  a16=(a16+a21);
  a11=(a11*a32);
  a16=(a16-a11);
  a16=(a16*a22);
  a40=(a36*a40);
  a16=(a16+a40);
  a41=(a41+a16);
  a16=(a34*a41);
  a19=(a35*a19);
  a36=(a5*a36);
  a19=(a19+a36);
  a36=(a25*a19);
  a16=(a16+a36);
  a2=(a2+a16);
  a41=(a18*a41);
  a19=(a5*a19);
  a41=(a41+a19);
  a17=(a17*a41);
  a2=(a2+a17);
  a17=(a2*a42);
  a28=(a28-a17);
  a23=(a23-a28);
  if (res[0]!=0) res[0][3]=a23;
  a1=sin(a1);
  a0=(a0*a1);
  a23=(a5*a0);
  a28=(a18*a1);
  a17=(a31*a28);
  a23=(a23-a17);
  a17=(a2*a28);
  a41=(a5*a38);
  a17=(a17+a41);
  a23=(a23+a17);
  if (res[0]!=0) res[0][4]=a23;
  a23=(a8*a28);
  a17=(a0*a42);
  a23=(a23-a17);
  a17=(a38*a42);
  a41=(a3*a28);
  a17=(a17+a41);
  a23=(a23-a17);
  if (res[0]!=0) res[0][5]=a23;
  a23=cos(a6);
  a17=(a5*a23);
  a41=(a10*a17);
  a6=sin(a6);
  a19=(a1*a6);
  a41=(a41-a19);
  a19=(a4*a41);
  a8=(a8+a19);
  a19=(a35*a23);
  a16=(a27*a19);
  a16=(a16+a45);
  a45=(a8*a16);
  a36=(a4*a19);
  a31=(a31+a36);
  a36=(a27*a41);
  a40=(a13*a42);
  a36=(a36+a40);
  a40=(a31*a36);
  a45=(a45-a40);
  a40=(a3*a16);
  a22=(a2*a36);
  a40=(a40-a22);
  a45=(a45-a40);
  if (res[0]!=0) res[0][6]=a45;
  a45=(a10*a6);
  a17=(a1*a17);
  a45=(a45+a17);
  a4=(a4*a45);
  a0=(a0+a4);
  a4=(a0*a16);
  a27=(a27*a45);
  a17=(a13*a28);
  a27=(a27+a17);
  a17=(a31*a27);
  a4=(a4-a17);
  a17=(a2*a27);
  a40=(a38*a16);
  a17=(a17+a40);
  a4=(a4+a17);
  if (res[0]!=0) res[0][7]=a4;
  a4=(a8*a27);
  a17=(a0*a36);
  a4=(a4-a17);
  a17=(a38*a36);
  a40=(a3*a27);
  a17=(a17+a40);
  a4=(a4-a17);
  if (res[0]!=0) res[0][8]=a4;
  a4=cos(a14);
  a17=(a13*a4);
  a40=(a41*a17);
  a22=(a20*a4);
  a11=(a42*a22);
  a40=(a40+a11);
  a11=(a1*a23);
  a32=(a5*a6);
  a21=(a10*a32);
  a11=(a11+a21);
  a14=sin(a14);
  a21=(a11*a14);
  a40=(a40-a21);
  a21=(a12*a40);
  a44=(a26*a36);
  a21=(a21+a44);
  a8=(a8+a21);
  a21=(a19*a17);
  a44=(a5*a22);
  a21=(a21+a44);
  a6=(a35*a6);
  a44=(a6*a14);
  a21=(a21-a44);
  a44=(a18*a21);
  a43=(a5*a16);
  a44=(a44+a43);
  a43=(a8*a44);
  a33=(a12*a21);
  a7=(a26*a16);
  a33=(a33+a7);
  a31=(a31+a33);
  a33=(a18*a40);
  a7=(a5*a36);
  a33=(a33+a7);
  a7=(a31*a33);
  a43=(a43-a7);
  a7=(a3*a44);
  a30=(a2*a33);
  a7=(a7-a30);
  a43=(a43-a7);
  if (res[0]!=0) res[0][9]=a43;
  a10=(a10*a23);
  a1=(a1*a32);
  a10=(a10-a1);
  a1=(a10*a14);
  a17=(a45*a17);
  a22=(a28*a22);
  a17=(a17+a22);
  a1=(a1+a17);
  a12=(a12*a1);
  a26=(a26*a27);
  a12=(a12+a26);
  a0=(a0+a12);
  a12=(a0*a44);
  a26=(a18*a1);
  a17=(a5*a27);
  a26=(a26+a17);
  a17=(a31*a26);
  a12=(a12-a17);
  a17=(a2*a26);
  a22=(a38*a44);
  a17=(a17+a22);
  a12=(a12+a17);
  if (res[0]!=0) res[0][10]=a12;
  a12=(a8*a26);
  a17=(a0*a33);
  a12=(a12-a17);
  a17=(a38*a33);
  a22=(a3*a26);
  a17=(a17+a22);
  a12=(a12-a17);
  if (res[0]!=0) res[0][11]=a12;
  a11=(a11*a4);
  a13=(a13*a14);
  a41=(a41*a13);
  a20=(a20*a14);
  a42=(a42*a20);
  a41=(a41+a42);
  a11=(a11+a41);
  a41=cos(a9);
  a42=(a11*a41);
  a9=sin(a9);
  a14=(a5*a9);
  a12=(a40*a14);
  a17=(a35*a9);
  a22=(a36*a17);
  a12=(a12+a22);
  a42=(a42+a12);
  a12=(a25*a42);
  a22=(a37*a33);
  a12=(a12+a22);
  a8=(a8+a12);
  a6=(a6*a4);
  a19=(a19*a13);
  a12=(a5*a20);
  a19=(a19+a12);
  a6=(a6+a19);
  a19=(a6*a41);
  a12=(a21*a14);
  a22=(a16*a17);
  a12=(a12+a22);
  a19=(a19+a12);
  a12=(a35*a19);
  a22=(a5*a44);
  a12=(a12+a22);
  a22=(a8*a12);
  a32=(a25*a19);
  a23=(a37*a44);
  a32=(a32+a23);
  a31=(a31+a32);
  a32=(a35*a42);
  a23=(a5*a33);
  a32=(a32+a23);
  a23=(a31*a32);
  a22=(a22-a23);
  a23=(a3*a12);
  a43=(a2*a32);
  a23=(a23-a43);
  a22=(a22-a23);
  if (res[0]!=0) res[0][12]=a22;
  a10=(a10*a4);
  a45=(a45*a13);
  a28=(a28*a20);
  a45=(a45+a28);
  a10=(a10-a45);
  a45=(a10*a41);
  a14=(a1*a14);
  a17=(a27*a17);
  a14=(a14+a17);
  a45=(a45-a14);
  a14=(a35*a45);
  a17=(a5*a26);
  a14=(a14-a17);
  a17=(a31*a14);
  a28=(a25*a45);
  a37=(a37*a26);
  a28=(a28-a37);
  a28=(a28-a0);
  a0=(a28*a12);
  a17=(a17-a0);
  a0=(a2*a14);
  a37=(a38*a12);
  a0=(a0-a37);
  a17=(a17-a0);
  if (res[0]!=0) res[0][13]=a17;
  a17=(a28*a32);
  a0=(a8*a14);
  a17=(a17-a0);
  a0=(a38*a32);
  a37=(a3*a14);
  a0=(a0-a37);
  a17=(a17-a0);
  if (res[0]!=0) res[0][14]=a17;
  a17=cos(a39);
  a0=(a5*a17);
  a42=(a42*a0);
  a37=(a5*a41);
  a40=(a40*a37);
  a35=(a35*a41);
  a36=(a36*a35);
  a40=(a40+a36);
  a11=(a11*a9);
  a40=(a40-a11);
  a39=sin(a39);
  a40=(a40*a39);
  a17=(a18*a17);
  a33=(a33*a17);
  a40=(a40+a33);
  a42=(a42+a40);
  a40=(a34*a42);
  a33=(a25*a32);
  a40=(a40+a33);
  a8=(a8+a40);
  a19=(a19*a0);
  a21=(a21*a37);
  a16=(a16*a35);
  a21=(a21+a16);
  a6=(a6*a9);
  a21=(a21-a6);
  a21=(a21*a39);
  a44=(a44*a17);
  a21=(a21+a44);
  a19=(a19+a21);
  a21=(a18*a19);
  a44=(a5*a12);
  a21=(a21+a44);
  a44=(a8*a21);
  a19=(a34*a19);
  a12=(a25*a12);
  a19=(a19+a12);
  a31=(a31+a19);
  a42=(a18*a42);
  a32=(a5*a32);
  a42=(a42+a32);
  a32=(a31*a42);
  a44=(a44-a32);
  a32=(a3*a21);
  a19=(a2*a42);
  a32=(a32-a19);
  a44=(a44-a32);
  if (res[0]!=0) res[0][15]=a44;
  a45=(a45*a0);
  a10=(a10*a9);
  a1=(a1*a37);
  a27=(a27*a35);
  a1=(a1+a27);
  a10=(a10+a1);
  a10=(a10*a39);
  a26=(a26*a17);
  a10=(a10+a26);
  a45=(a45-a10);
  a18=(a18*a45);
  a5=(a5*a14);
  a18=(a18+a5);
  a31=(a31*a18);
  a34=(a34*a45);
  a25=(a25*a14);
  a34=(a34+a25);
  a28=(a28+a34);
  a34=(a28*a21);
  a31=(a31-a34);
  a2=(a2*a18);
  a21=(a38*a21);
  a2=(a2-a21);
  a31=(a31-a2);
  if (res[0]!=0) res[0][16]=a31;
  a28=(a28*a42);
  a8=(a8*a18);
  a28=(a28-a8);
  a38=(a38*a42);
  a3=(a3*a18);
  a38=(a38-a3);
  a28=(a28-a38);
  if (res[0]!=0) res[0][17]=a28;
  return 0;
}

CASADI_SYMBOL_EXPORT int J(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int J_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int J_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int J_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_release(int mem) {
}

CASADI_SYMBOL_EXPORT void J_incref(void) {
}

CASADI_SYMBOL_EXPORT void J_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int J_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int J_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real J_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int J_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif