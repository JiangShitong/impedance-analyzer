/*

 * Nonlinear_solve.c

 *

 *  Created on: 2019Äê4ÔÂ7ÈÕ

 *      Author: Administrator

 */

#include <stdio.h>

#include <math.h>



#define Matrix_Dimension 3	//attention!!

//void inverse_complex_matrix(double in_real[Matrix_Dimension][Matrix_Dimension], double in_imag[Matrix_Dimension][Matrix_Dimension], double out_real[Matrix_Dimension][Matrix_Dimension], double out_imag[Matrix_Dimension][Matrix_Dimension]);
void inverse_complex_matrix(double in_real[Matrix_Dimension][Matrix_Dimension], double in_imag[Matrix_Dimension][Matrix_Dimension], double out_real[Matrix_Dimension][Matrix_Dimension], double out_imag[Matrix_Dimension][Matrix_Dimension], int sovle_dimension);

void complex_product(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);

void complex_plus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);

//void nonlinear_solve(double *val_X, double *val_Y, double *result_X, double *result_Y, double *coef_X, double *coef_Y);
void nonlinear_solve(double *val_X, double *val_Y, double *result_X, double *result_Y, double *coef_X, double *coef_Y, int sovle_dimension);


void inverse_complex_matrix(double in_real[Matrix_Dimension][Matrix_Dimension], double in_imag[Matrix_Dimension][Matrix_Dimension], double out_real[Matrix_Dimension][Matrix_Dimension], double out_imag[Matrix_Dimension][Matrix_Dimension], int sovle_dimension)

{

	int cnt_1st_cir,cnt_2nd_cir,cnt_3rd_cir;

	int cnt_temp;

	double temp;

	double augmented_matrix[3*Matrix_Dimension][3*Matrix_Dimension]={0};



	/**********	generate the augmented_matrix	*****/

	//the 1st block row£º left is real matrix, middle is negative imaginative matrix, right is unit matrix

    for(cnt_1st_cir=0;cnt_1st_cir<sovle_dimension;cnt_1st_cir++)

	{

		//place the real matrix

		for(cnt_2nd_cir=0;cnt_2nd_cir<sovle_dimension;cnt_2nd_cir++)

			augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=in_real[cnt_1st_cir][cnt_2nd_cir];



		//place the negative imaginative matrix

		for(cnt_2nd_cir=sovle_dimension;cnt_2nd_cir<2*sovle_dimension;cnt_2nd_cir++)

			augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=-1.0*in_imag[cnt_1st_cir][cnt_2nd_cir-sovle_dimension];



		//place the unit matrix

		augmented_matrix[cnt_1st_cir][cnt_1st_cir+2*sovle_dimension]=1;

	}



	//the 2nd block row£º left is imaginative matrix, middle is real matrix, right is zero matrix

	for(cnt_1st_cir=sovle_dimension;cnt_1st_cir<2*sovle_dimension;cnt_1st_cir++)

	{

		//place the imaginative matrix

		for(cnt_2nd_cir=0;cnt_2nd_cir<sovle_dimension;cnt_2nd_cir++)

			augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=in_imag[cnt_1st_cir-sovle_dimension][cnt_2nd_cir];



		//place the real matrix

		for(cnt_2nd_cir=sovle_dimension;cnt_2nd_cir<2*sovle_dimension;cnt_2nd_cir++)

			augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=in_real[cnt_1st_cir-sovle_dimension][cnt_2nd_cir-sovle_dimension];

	}



	/**********	Gaussian substitution	*****/

	//find the greatest element of each diagonal comlumn, exchange its row to diagonal row, generate upper triangular matrix

	for(cnt_1st_cir=0;cnt_1st_cir<2*sovle_dimension;cnt_1st_cir++)

	{

		//find the greatest element of each diagonal comlumn

        for(cnt_2nd_cir=cnt_1st_cir;cnt_2nd_cir<2*sovle_dimension;cnt_2nd_cir++)

		{

			if(fabs(augmented_matrix[cnt_2nd_cir][cnt_1st_cir])>fabs(augmented_matrix[cnt_1st_cir][cnt_1st_cir]))

				cnt_temp=cnt_2nd_cir;

			else

				cnt_temp=cnt_1st_cir;

		}



		//shift row

		for(cnt_2nd_cir=0;cnt_2nd_cir<3*sovle_dimension;cnt_2nd_cir++)

		{

			temp=augmented_matrix[cnt_1st_cir][cnt_2nd_cir];

            augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=augmented_matrix[cnt_temp][cnt_2nd_cir];

		    augmented_matrix[cnt_temp][cnt_2nd_cir]=temp;

		}



		//generate upper triangular matrix

	    for(cnt_2nd_cir=cnt_1st_cir+1;cnt_2nd_cir<2*sovle_dimension;cnt_2nd_cir++)

			for(cnt_3rd_cir=3*sovle_dimension-1;cnt_3rd_cir>=cnt_1st_cir;cnt_3rd_cir--)

				augmented_matrix[cnt_2nd_cir][cnt_3rd_cir]=augmented_matrix[cnt_2nd_cir][cnt_3rd_cir]-augmented_matrix[cnt_1st_cir][cnt_3rd_cir]*augmented_matrix[cnt_2nd_cir][cnt_1st_cir]/augmented_matrix[cnt_1st_cir][cnt_1st_cir];

	}



	//generate diagonal matrix

	for(cnt_1st_cir=2*sovle_dimension-1;cnt_1st_cir>0;cnt_1st_cir--)

		for(cnt_2nd_cir=0;cnt_2nd_cir<cnt_1st_cir;cnt_2nd_cir++)

			for(cnt_3rd_cir=3*sovle_dimension-1;cnt_3rd_cir>=0;cnt_3rd_cir--)

				augmented_matrix[cnt_2nd_cir][cnt_3rd_cir]=augmented_matrix[cnt_2nd_cir][cnt_3rd_cir]-augmented_matrix[cnt_1st_cir][cnt_3rd_cir]*augmented_matrix[cnt_2nd_cir][cnt_1st_cir]/augmented_matrix[cnt_1st_cir][cnt_1st_cir];



	//generate the unit matrix

	for(cnt_1st_cir=0;cnt_1st_cir<2*sovle_dimension;cnt_1st_cir++)

		for(cnt_2nd_cir=3*sovle_dimension-1;cnt_2nd_cir>=0;cnt_2nd_cir--)

			augmented_matrix[cnt_1st_cir][cnt_2nd_cir]=augmented_matrix[cnt_1st_cir][cnt_2nd_cir]/augmented_matrix[cnt_1st_cir][cnt_1st_cir];



	//obtain the inverse matrix of real and imaginative part

	for(cnt_1st_cir=0;cnt_1st_cir<sovle_dimension;cnt_1st_cir++)

		for(cnt_2nd_cir=0;cnt_2nd_cir<sovle_dimension;cnt_2nd_cir++)

		{

			out_real[cnt_1st_cir][cnt_2nd_cir]=augmented_matrix[cnt_1st_cir][cnt_2nd_cir+2*sovle_dimension];

			out_imag[cnt_1st_cir][cnt_2nd_cir]=augmented_matrix[cnt_1st_cir+sovle_dimension][cnt_2nd_cir+2*sovle_dimension];

		}

}



//the function of complex_number multiplication: C=A*B -> C_X+j*C_Y = (A_X+j*A_Y)*(B_X+j*B_Y)

void complex_product(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y)

{

	double temp_X, temp_Y;



	temp_X = A_X*B_X - A_Y*B_Y;

	temp_Y = A_X*B_Y + A_Y*B_X;



	*C_X = temp_X;

	*C_Y = temp_Y;

}

void complex_divide(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y)

{

	double temp_X, temp_Y, temp;

    temp = B_X*B_X + B_Y*B_Y;

	temp_X = A_X*B_X + A_Y*B_Y;

	temp_Y = A_Y*B_X - A_X*B_Y;



	*C_X = temp_X / temp;

	*C_Y = temp_Y / temp;

}



//the function of complex_number addition: C=A+B -> C_X+j*C_Y = (A_X+j*A_Y)+(B_X+j*B_Y)

void complex_plus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y)

{

	double temp_X, temp_Y;



	temp_X = A_X + B_X;

	temp_Y = A_Y + B_Y;



	*C_X = temp_X;

	*C_Y = temp_Y;

}


void complex_minus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y)

{

	double temp_X, temp_Y;



	temp_X = A_X - B_X;

	temp_Y = A_Y - B_Y;



	*C_X = temp_X;

	*C_Y = temp_Y;

}

//solve the complex_number of nonlinear coefficient [3*3 matrix]

void nonlinear_solve(double *val_X, double *val_Y, double *result_X, double *result_Y, double *coef_X, double *coef_Y, int sovle_dimension)

{

	double val_real_matrix[Matrix_Dimension][Matrix_Dimension];

	double val_imag_matrix[Matrix_Dimension][Matrix_Dimension];

	int cnt_row, cnt_column;

	double val_temp_X, val_temp_Y;		//the intermediate value for operation

	double val_matrix_X, val_matrix_Y;	//pick up the correspond element from the matrix to subfunction

	double inv_real_matrix[Matrix_Dimension][Matrix_Dimension];

	double inv_imag_matrix[Matrix_Dimension][Matrix_Dimension];



	/*********	fulfill the multi_order matrix	*****/

	for(cnt_row=0;cnt_row<sovle_dimension;cnt_row++)

	{

		for(cnt_column=0;cnt_column<sovle_dimension;cnt_column++)

		{

			if(cnt_column == 0)	//for 0 power

			{

				val_real_matrix[cnt_row][cnt_column] = 1.0;

				val_imag_matrix[cnt_row][cnt_column] = 0.0;

				val_temp_X = *(val_X+cnt_row);

				val_temp_Y = *(val_Y+cnt_row);

			}



			else

			{

				val_real_matrix[cnt_row][cnt_column] = val_temp_X;

				val_imag_matrix[cnt_row][cnt_column] = val_temp_Y;

                val_matrix_X = val_X[cnt_row];

                val_matrix_Y = val_Y[cnt_row];

				//complex_product(val_temp_X, val_temp_Y, *(val_X+cnt_row), *(val_Y+cnt_row), val_temp_X, val_temp_Y);

                complex_product(val_temp_X, val_temp_Y, val_matrix_X, val_matrix_Y, &val_temp_X, &val_temp_Y);

			}



		}

	}



	inverse_complex_matrix(val_real_matrix, val_imag_matrix, inv_real_matrix, inv_imag_matrix, sovle_dimension);	//sovle out the inverse matrix of the multi_order matrix



	for(cnt_row=0;cnt_row<sovle_dimension;cnt_row++)

	{

		for(cnt_column=0;cnt_column<sovle_dimension;cnt_column++)

		{

			if(cnt_column == 0)	//initialize the  coef_X and coef_Y+

			{

				*(coef_X+cnt_row) = 0;

				*(coef_Y+cnt_row) = 0;

			}



			val_matrix_X = inv_real_matrix[cnt_row][cnt_column];

			val_matrix_Y = inv_imag_matrix[cnt_row][cnt_column];


			complex_product(*(result_X+cnt_column), *(result_Y+cnt_column), val_matrix_X, val_matrix_Y, &val_temp_X, &val_temp_Y);



			complex_plus(*(coef_X+cnt_row), *(coef_Y+cnt_row), val_temp_X, val_temp_Y, coef_X+cnt_row, coef_Y+cnt_row);

		}



	}

}



//void main(void)
//
//{
//
//    int cnt;
//
//
//
//    double Lp_matrix_X[Matrix_Dimension] = {-0.181212797760963, -0.362432999999999, -0.544654599999999, -0.729539191722869};
//
//    double Lp_matrix_Y[Matrix_Dimension] = {-0.105972799658775, -0.2096254, -0.311218199999999, -0.411731600761413};
//
//
//
//    double Vr_matrix_X[Matrix_Dimension] = {0.2278596, 0.455055399999999, 0.682515597343445, 0.912088394165039};
//
//    double Vr_matrix_Y[Matrix_Dimension] = {0.0356282, 0.0676046999999999, 0.09728772044181825, 0.125699001550674};
//
//
//
//    double coef_matrix_X[Matrix_Dimension]={};
//
//    double coef_matrix_Y[Matrix_Dimension]={};
//
//
//
//    nonlinear_solve(Lp_matrix_X, Lp_matrix_Y, Vr_matrix_X, Vr_matrix_Y, coef_matrix_X, coef_matrix_Y);
//
//
//
//    for(cnt=0;cnt<Matrix_Dimension;cnt++)
//
//    {
//
//        printf("%f\t",coef_matrix_X[cnt]);
//
//        printf("%f\t",coef_matrix_Y[cnt]);
//
//        printf("\n");
//
//    }
//
//
//
//}
