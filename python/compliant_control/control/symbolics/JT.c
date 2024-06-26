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
  #define CASADI_PREFIX(ID) JT_ ## ID
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
static const casadi_int casadi_s1[24] = {6, 3, 0, 6, 12, 18, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};

/* JT:(i0[6])->(o0[6x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a8, a9;
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
  a17=1.5600000000000000e-01;
  a42=(a18*a42);
  a31=(a5*a31);
  a42=(a42+a31);
  a42=(a17*a42);
  a3=(a3+a42);
  a42=(-a3);
  if (res[0]!=0) res[0][0]=a42;
  a42=cos(a1);
  a31=(a0*a42);
  a38=(a5*a31);
  a28=2.4325000000000002e-01;
  a45=(a18*a42);
  a46=(a28*a45);
  a38=(a38-a46);
  a46=(a5*a3);
  a47=(a35*a7);
  a48=(a4*a47);
  a48=(a28+a48);
  a49=(a47*a16);
  a50=(a5*a21);
  a49=(a49+a50);
  a50=(a35*a11);
  a51=(a50*a24);
  a49=(a49-a51);
  a51=(a12*a49);
  a52=(a27*a47);
  a53=3.6732051035389906e-06;
  a52=(a52+a53);
  a54=(a26*a52);
  a51=(a51+a54);
  a48=(a48+a51);
  a50=(a50*a15);
  a47=(a47*a29);
  a51=(a5*a30);
  a47=(a47+a51);
  a50=(a50+a47);
  a47=(a50*a19);
  a51=(a49*a33);
  a54=(a52*a36);
  a51=(a51+a54);
  a47=(a47+a51);
  a51=(a25*a47);
  a54=(a18*a49);
  a55=(a5*a52);
  a54=(a54+a55);
  a55=(a37*a54);
  a51=(a51+a55);
  a48=(a48+a51);
  a51=(a47*a41);
  a49=(a49*a43);
  a52=(a52*a44);
  a49=(a49+a52);
  a50=(a50*a32);
  a49=(a49-a50);
  a49=(a49*a22);
  a50=(a54*a40);
  a49=(a49+a50);
  a51=(a51+a49);
  a49=(a34*a51);
  a47=(a35*a47);
  a54=(a5*a54);
  a47=(a47+a54);
  a54=(a25*a47);
  a49=(a49+a54);
  a48=(a48+a49);
  a51=(a18*a51);
  a47=(a5*a47);
  a51=(a51+a47);
  a51=(a17*a51);
  a48=(a48+a51);
  a51=(a48*a45);
  a46=(a46-a51);
  a38=(a38-a46);
  if (res[0]!=0) res[0][1]=a38;
  a38=cos(a6);
  a46=(a5*a38);
  a51=(a42*a46);
  a1=sin(a1);
  a6=sin(a6);
  a47=(a1*a6);
  a51=(a51-a47);
  a47=(a4*a51);
  a47=(a31+a47);
  a49=(a35*a38);
  a54=(a27*a49);
  a54=(a54+a53);
  a53=(a47*a54);
  a50=(a4*a49);
  a50=(a28+a50);
  a52=(a27*a51);
  a55=(a13*a45);
  a52=(a52+a55);
  a55=(a50*a52);
  a53=(a53-a55);
  a55=(a3*a54);
  a56=(a48*a52);
  a55=(a55-a56);
  a53=(a53-a55);
  if (res[0]!=0) res[0][2]=a53;
  a53=cos(a14);
  a55=(a13*a53);
  a56=(a51*a55);
  a57=(a20*a53);
  a58=(a45*a57);
  a56=(a56+a58);
  a58=(a1*a38);
  a59=(a5*a6);
  a60=(a42*a59);
  a58=(a58+a60);
  a14=sin(a14);
  a60=(a58*a14);
  a56=(a56-a60);
  a60=(a12*a56);
  a61=(a26*a52);
  a60=(a60+a61);
  a60=(a47+a60);
  a61=(a49*a55);
  a62=(a5*a57);
  a61=(a61+a62);
  a62=(a35*a6);
  a63=(a62*a14);
  a61=(a61-a63);
  a63=(a18*a61);
  a64=(a5*a54);
  a63=(a63+a64);
  a64=(a60*a63);
  a65=(a12*a61);
  a66=(a26*a54);
  a65=(a65+a66);
  a65=(a50+a65);
  a66=(a18*a56);
  a67=(a5*a52);
  a66=(a66+a67);
  a67=(a65*a66);
  a64=(a64-a67);
  a67=(a3*a63);
  a68=(a48*a66);
  a67=(a67-a68);
  a64=(a64-a67);
  if (res[0]!=0) res[0][3]=a64;
  a58=(a58*a53);
  a64=(a13*a14);
  a51=(a51*a64);
  a20=(a20*a14);
  a67=(a45*a20);
  a51=(a51+a67);
  a58=(a58+a51);
  a51=cos(a9);
  a67=(a58*a51);
  a9=sin(a9);
  a68=(a5*a9);
  a69=(a56*a68);
  a70=(a35*a9);
  a71=(a52*a70);
  a69=(a69+a71);
  a67=(a67+a69);
  a69=(a25*a67);
  a71=(a37*a66);
  a69=(a69+a71);
  a69=(a60+a69);
  a62=(a62*a53);
  a49=(a49*a64);
  a71=(a5*a20);
  a49=(a49+a71);
  a62=(a62+a49);
  a49=(a62*a51);
  a71=(a61*a68);
  a72=(a54*a70);
  a71=(a71+a72);
  a49=(a49+a71);
  a71=(a35*a49);
  a72=(a5*a63);
  a71=(a71+a72);
  a72=(a69*a71);
  a73=(a25*a49);
  a74=(a37*a63);
  a73=(a73+a74);
  a73=(a65+a73);
  a74=(a35*a67);
  a75=(a5*a66);
  a74=(a74+a75);
  a75=(a73*a74);
  a72=(a72-a75);
  a75=(a3*a71);
  a76=(a48*a74);
  a75=(a75-a76);
  a72=(a72-a75);
  if (res[0]!=0) res[0][4]=a72;
  a72=cos(a39);
  a75=(a5*a72);
  a67=(a67*a75);
  a76=(a5*a51);
  a56=(a56*a76);
  a77=(a35*a51);
  a78=(a52*a77);
  a56=(a56+a78);
  a58=(a58*a9);
  a56=(a56-a58);
  a39=sin(a39);
  a56=(a56*a39);
  a72=(a18*a72);
  a58=(a66*a72);
  a56=(a56+a58);
  a67=(a67+a56);
  a56=(a34*a67);
  a58=(a25*a74);
  a56=(a56+a58);
  a56=(a69+a56);
  a49=(a49*a75);
  a61=(a61*a76);
  a58=(a54*a77);
  a61=(a61+a58);
  a62=(a62*a9);
  a61=(a61-a62);
  a61=(a61*a39);
  a62=(a63*a72);
  a61=(a61+a62);
  a49=(a49+a61);
  a61=(a18*a49);
  a62=(a5*a71);
  a61=(a61+a62);
  a62=(a56*a61);
  a49=(a34*a49);
  a58=(a25*a71);
  a49=(a49+a58);
  a49=(a73+a49);
  a67=(a18*a67);
  a58=(a5*a74);
  a67=(a67+a58);
  a58=(a49*a67);
  a62=(a62-a58);
  a58=(a3*a61);
  a78=(a48*a67);
  a58=(a58-a78);
  a62=(a62-a58);
  if (res[0]!=0) res[0][5]=a62;
  a7=(a2*a7);
  a23=(a10*a23);
  a7=(a7-a23);
  a15=(a7*a15);
  a2=(a2*a11);
  a8=(a10*a8);
  a2=(a2+a8);
  a29=(a2*a29);
  a8=(a18*a10);
  a30=(a8*a30);
  a29=(a29+a30);
  a15=(a15-a29);
  a19=(a15*a19);
  a7=(a7*a24);
  a16=(a2*a16);
  a21=(a8*a21);
  a16=(a16+a21);
  a7=(a7+a16);
  a33=(a7*a33);
  a16=(a27*a2);
  a8=(a13*a8);
  a16=(a16+a8);
  a36=(a16*a36);
  a33=(a33+a36);
  a19=(a19-a33);
  a33=(a25*a19);
  a36=(a18*a7);
  a8=(a5*a16);
  a36=(a36+a8);
  a8=(a37*a36);
  a33=(a33-a8);
  a10=(a0*a10);
  a2=(a4*a2);
  a10=(a10+a2);
  a2=(a12*a7);
  a8=(a26*a16);
  a2=(a2+a8);
  a10=(a10+a2);
  a33=(a33-a10);
  a41=(a19*a41);
  a15=(a15*a32);
  a7=(a7*a43);
  a16=(a16*a44);
  a7=(a7+a16);
  a15=(a15+a7);
  a15=(a15*a22);
  a40=(a36*a40);
  a15=(a15+a40);
  a41=(a41-a15);
  a15=(a34*a41);
  a19=(a35*a19);
  a36=(a5*a36);
  a19=(a19-a36);
  a36=(a25*a19);
  a15=(a15+a36);
  a33=(a33+a15);
  a41=(a18*a41);
  a19=(a5*a19);
  a41=(a41+a19);
  a17=(a17*a41);
  a33=(a33+a17);
  if (res[0]!=0) res[0][6]=a33;
  a0=(a0*a1);
  a17=(a5*a0);
  a41=(a18*a1);
  a28=(a28*a41);
  a17=(a17-a28);
  a28=(a48*a41);
  a19=(a5*a33);
  a28=(a28+a19);
  a17=(a17+a28);
  if (res[0]!=0) res[0][7]=a17;
  a6=(a42*a6);
  a46=(a1*a46);
  a6=(a6+a46);
  a4=(a4*a6);
  a4=(a0+a4);
  a46=(a4*a54);
  a27=(a27*a6);
  a13=(a13*a41);
  a27=(a27+a13);
  a50=(a50*a27);
  a46=(a46-a50);
  a50=(a48*a27);
  a54=(a33*a54);
  a50=(a50+a54);
  a46=(a46+a50);
  if (res[0]!=0) res[0][8]=a46;
  a42=(a42*a38);
  a1=(a1*a59);
  a42=(a42-a1);
  a14=(a42*a14);
  a55=(a6*a55);
  a57=(a41*a57);
  a55=(a55+a57);
  a14=(a14+a55);
  a12=(a12*a14);
  a26=(a26*a27);
  a12=(a12+a26);
  a12=(a4+a12);
  a26=(a12*a63);
  a55=(a18*a14);
  a57=(a5*a27);
  a55=(a55+a57);
  a65=(a65*a55);
  a26=(a26-a65);
  a65=(a48*a55);
  a63=(a33*a63);
  a65=(a65+a63);
  a26=(a26+a65);
  if (res[0]!=0) res[0][9]=a26;
  a42=(a42*a53);
  a6=(a6*a64);
  a20=(a41*a20);
  a6=(a6+a20);
  a42=(a42-a6);
  a51=(a42*a51);
  a68=(a14*a68);
  a70=(a27*a70);
  a68=(a68+a70);
  a51=(a51-a68);
  a35=(a35*a51);
  a68=(a5*a55);
  a35=(a35-a68);
  a73=(a73*a35);
  a68=(a25*a51);
  a37=(a37*a55);
  a68=(a68-a37);
  a68=(a68-a12);
  a37=(a68*a71);
  a73=(a73-a37);
  a37=(a48*a35);
  a71=(a33*a71);
  a37=(a37-a71);
  a73=(a73-a37);
  if (res[0]!=0) res[0][10]=a73;
  a51=(a51*a75);
  a42=(a42*a9);
  a14=(a14*a76);
  a77=(a27*a77);
  a14=(a14+a77);
  a42=(a42+a14);
  a42=(a42*a39);
  a72=(a55*a72);
  a42=(a42+a72);
  a51=(a51-a42);
  a18=(a18*a51);
  a5=(a5*a35);
  a18=(a18+a5);
  a49=(a49*a18);
  a34=(a34*a51);
  a25=(a25*a35);
  a34=(a34+a25);
  a34=(a68+a34);
  a25=(a34*a61);
  a49=(a49-a25);
  a48=(a48*a18);
  a61=(a33*a61);
  a48=(a48-a61);
  a49=(a49-a48);
  if (res[0]!=0) res[0][11]=a49;
  a49=0.;
  if (res[0]!=0) res[0][12]=a49;
  a31=(a31*a41);
  a0=(a0*a45);
  a31=(a31-a0);
  a45=(a33*a45);
  a41=(a3*a41);
  a45=(a45+a41);
  a31=(a31-a45);
  if (res[0]!=0) res[0][13]=a31;
  a47=(a47*a27);
  a4=(a4*a52);
  a47=(a47-a4);
  a52=(a33*a52);
  a27=(a3*a27);
  a52=(a52+a27);
  a47=(a47-a52);
  if (res[0]!=0) res[0][14]=a47;
  a60=(a60*a55);
  a12=(a12*a66);
  a60=(a60-a12);
  a66=(a33*a66);
  a55=(a3*a55);
  a66=(a66+a55);
  a60=(a60-a66);
  if (res[0]!=0) res[0][15]=a60;
  a68=(a68*a74);
  a69=(a69*a35);
  a68=(a68-a69);
  a74=(a33*a74);
  a35=(a3*a35);
  a74=(a74-a35);
  a68=(a68-a74);
  if (res[0]!=0) res[0][16]=a68;
  a34=(a34*a67);
  a56=(a56*a18);
  a34=(a34-a56);
  a33=(a33*a67);
  a3=(a3*a18);
  a33=(a33-a3);
  a34=(a34-a33);
  if (res[0]!=0) res[0][17]=a34;
  return 0;
}

CASADI_SYMBOL_EXPORT int JT(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int JT_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int JT_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void JT_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int JT_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void JT_release(int mem) {
}

CASADI_SYMBOL_EXPORT void JT_incref(void) {
}

CASADI_SYMBOL_EXPORT void JT_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int JT_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int JT_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real JT_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* JT_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* JT_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* JT_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* JT_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int JT_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
