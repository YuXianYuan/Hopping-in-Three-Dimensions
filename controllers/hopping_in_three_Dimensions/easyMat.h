/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为一个简单的矩阵运算库头文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _EASYMAT_H_
#define _EASYMAT_H_

#include <stdint.h>

//-----------------------------------------------------------typedef
/*
矩阵 类型
*/
typedef struct
{
  uint16_t row;
  uint16_t col;
  double** data;
}matTypeDef;
//-----------------------------------------------------------function
extern void easyMat_create    (matTypeDef* mat, uint16_t row, uint16_t col)              ;
extern void easyMat_free      (matTypeDef* mat)                                          ;
extern void easyMat_init      (matTypeDef* mat, double* data)                            ;
extern void easyMat_clear     (matTypeDef* mat)                                          ;
extern void easyMat_eye       (matTypeDef* mat)                                          ;
extern void easyMat_add       (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_sub       (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_mult      (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_trans     (matTypeDef* dstMat, matTypeDef* srcMat)                   ;
extern void easyMat_copy      (matTypeDef* dstMat, matTypeDef* srcMat)                   ;
//extern void easyMat_inv       (matTypeDef* dstMat, matTypeDef* srcMat)                  ;
extern void easyMat_rotX      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_rotY      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_rotZ      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_RPY       (matTypeDef* outRot, double roll, double pitch, double yaw);



#endif

