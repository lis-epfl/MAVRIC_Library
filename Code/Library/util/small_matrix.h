#ifndef SMALL_MATRIX_H_
#define SMALL_MATRIX_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct matrix_2x2_t {
   float v[2][2];
} matrix_2x2_t; 


typedef struct vector_2_t {
   float v[2];
} vector_2_t;

static const matrix_2x2_t zero_2x2= 
   {.v={{0.0f, 0.0f}, 
        {0.0f, 0.0f}} };

static const matrix_2x2_t ones_2x2= 
   {.v={{1.0f, 1.0f}, 
        {1.0f, 1.0f}} };

static const matrix_2x2_t ident_2x2= 
   {.v={{1.0f, 0.0f}, 
        {0.0f, 1.0f}} };

// create diagonal matrix
matrix_2x2_t static inline diag_2x2(const vector_2_t v) {
   matrix_2x2_t result= zero_2x2;
   result.v[0][0]=v.v[0];result.v[1][1]=v.v[1];   return result;
}

// return vector of a row
vector_2_t static inline row2(const matrix_2x2_t m, int row) {
   vector_2_t  result= {.v={m.v[row][0], m.v[row][1]}};
   return result;
}

// return vector of a column
vector_2_t static inline col2(const matrix_2x2_t m, int col) {
   vector_2_t  result= {.v={m.v[0][col], m.v[1][col]}};
   return result;
}

// return matrix diagonal as a vector
vector_2_t static inline diag_vector2(const matrix_2x2_t m) {
   vector_2_t result= {.v={m.v[0][0], m.v[1][1]}};
   return result;
}

// transpose of a matrix
matrix_2x2_t static inline trans2(const matrix_2x2_t m) {
   matrix_2x2_t result= 
   {.v={m.v[0][0], m.v[1][0], 
        m.v[0][1], m.v[1][1]}};
   return result;
}

// matrix product for 2x2 matrices
matrix_2x2_t static inline mmul2(const matrix_2x2_t m1, const matrix_2x2_t m2) {
   matrix_2x2_t result= 
   {.v={m1.v[0][0]* m2.v[0][0]+m1.v[0][1]* m2.v[1][0], m1.v[0][0]* m2.v[0][1]+m1.v[0][1]* m2.v[1][1], 
        m1.v[1][0]* m2.v[0][0]+m1.v[1][1]* m2.v[1][0], m1.v[1][0]* m2.v[0][1]+m1.v[1][1]* m2.v[1][1]}};
   return result;
}

// scalar/matrix product for 2x2 matrices
matrix_2x2_t static inline smmul2(const float s, const matrix_2x2_t m) {
   matrix_2x2_t result= 
   {.v={s * m.v[0][0], s * m.v[0][1], 
        s * m.v[1][0], s * m.v[1][1]}};
   return result;
}

// matrix/vector product for 2x2 matrices
vector_2_t static inline mvmul2(const matrix_2x2_t m1, const vector_2_t vec) {
   vector_2_t result= 
   {.v={m1.v[0][0]* vec.v[0]+m1.v[0][1]* vec.v[1], 
        m1.v[1][0]* vec.v[0]+m1.v[1][1]* vec.v[1]}};
   return result;
}

// scalar/vector product for 2x2 vectors
vector_2_t static inline svmul2(const float s, const vector_2_t vec) {
   vector_2_t result= 
   {.v={s* vec.v[0], 
        s* vec.v[1]}};
   return result;
}

// tensor product for 2D vectors
matrix_2x2_t static inline tp2(const vector_2_t vec1, const vector_2_t vec2) {
   matrix_2x2_t result= 
   {.v={vec1.v[0]* vec2.v[0], vec1.v[0]* vec2.v[1], 
        vec1.v[1]* vec2.v[0], vec1.v[1]* vec2.v[1]}};
   return result;
}

// scalar product for 2D vectors
float static inline sp2(const vector_2_t vec1, const vector_2_t vec2) {
   return (vec1.v[0]* vec2.v[0]+vec1.v[1]* vec2.v[1]);
 }


// pointwise add for 2x2 matrices
matrix_2x2_t static inline madd2(const matrix_2x2_t m1, const matrix_2x2_t m2) {
   matrix_2x2_t result= 
   {.v={m1.v[0][0]+ m2.v[0][0], m1.v[0][1]+ m2.v[0][1], 
        m1.v[1][0]+ m2.v[1][0], m1.v[1][1]+ m2.v[1][1]}};
   return result;
}

// pointwise add for 2x2 vectors
vector_2_t static inline vadd2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1]}};
   return result;
}


// pointwise sub for 2x2 matrices
matrix_2x2_t static inline msub2(const matrix_2x2_t m1, const matrix_2x2_t m2) {
   matrix_2x2_t result= 
   {.v={m1.v[0][0]- m2.v[0][0], m1.v[0][1]- m2.v[0][1], 
        m1.v[1][0]- m2.v[1][0], m1.v[1][1]- m2.v[1][1]}};
   return result;
}

// pointwise sub for 2x2 vectors
vector_2_t static inline vsub2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1]}};
   return result;
}


// pointwise pwmul for 2x2 matrices
matrix_2x2_t static inline mpwmul2(const matrix_2x2_t m1, const matrix_2x2_t m2) {
   matrix_2x2_t result= 
   {.v={m1.v[0][0]* m2.v[0][0], m1.v[0][1]* m2.v[0][1], 
        m1.v[1][0]* m2.v[1][0], m1.v[1][1]* m2.v[1][1]}};
   return result;
}

// pointwise pwmul for 2x2 vectors
vector_2_t static inline vpwmul2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1]}};
   return result;
}


//squared frobenius norm  for 2x2 matrices
float static inline sqr_f_norm2(const matrix_2x2_t m) {
   float result= 
   m.v[0][0]* m.v[0][0] + m.v[0][1]* m.v[0][1] +
   m.v[1][0]* m.v[1][0] + m.v[1][1]* m.v[1][1];
   return result;
}

//sum of each row  for 2x2 matrices
vector_2_t static inline row_sum2(const matrix_2x2_t m) {
   vector_2_t result= {.v={
   m.v[0][0] + m.v[0][1] , 
   m.v[1][0] + m.v[1][1]}};
   return result;
}

//sum of each column  for 2x2 matrices
vector_2_t static inline col_sum2(const matrix_2x2_t m) {
   vector_2_t result= {.v={
   m.v[0][0] + m.v[1][0] , 
   m.v[0][1] + m.v[1][1]}};
   return result;
}

//sum for 2D vectors
float static inline sum2(const vector_2_t vec) {
   float result= vec.v[0] + vec.v[1];
   return result;
}

//trace for 2D matrices
float static inline trace2(const matrix_2x2_t m) {
   return sum2(diag_vector2(m));
}

//squared norm  for 2D vectors
float static inline sqr_norm2(const vector_2_t vec) {
   float result= vec.v[0]* vec.v[0] + vec.v[1]* vec.v[1];
   return result;
}


typedef struct matrix_3x3_t {
   float v[3][3];
} matrix_3x3_t; 


typedef struct vector_3_t {
   float v[3];
} vector_3_t;

static const matrix_3x3_t zero_3x3= 
   {.v={{0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f}} };

static const matrix_3x3_t ones_3x3= 
   {.v={{1.0f, 1.0f, 1.0f}, 
        {1.0f, 1.0f, 1.0f}, 
        {1.0f, 1.0f, 1.0f}} };

static const matrix_3x3_t ident_3x3= 
   {.v={{1.0f, 0.0f, 0.0f}, 
        {0.0f, 1.0f, 0.0f}, 
        {0.0f, 0.0f, 1.0f}} };

// create diagonal matrix
matrix_3x3_t static inline diag_3x3(const vector_3_t v) {
   matrix_3x3_t result= zero_3x3;
   result.v[0][0]=v.v[0];result.v[1][1]=v.v[1];result.v[2][2]=v.v[2];   return result;
}

// return vector of a row
vector_3_t static inline row3(const matrix_3x3_t m, int row) {
   vector_3_t  result= {.v={m.v[row][0], m.v[row][1], m.v[row][2]}};
   return result;
}

// return vector of a column
vector_3_t static inline col3(const matrix_3x3_t m, int col) {
   vector_3_t  result= {.v={m.v[0][col], m.v[1][col], m.v[2][col]}};
   return result;
}

// return matrix diagonal as a vector
vector_3_t static inline diag_vector3(const matrix_3x3_t m) {
   vector_3_t result= {.v={m.v[0][0], m.v[1][1], m.v[2][2]}};
   return result;
}

// transpose of a matrix
matrix_3x3_t static inline trans3(const matrix_3x3_t m) {
   matrix_3x3_t result= 
   {.v={m.v[0][0], m.v[1][0], m.v[2][0], 
        m.v[0][1], m.v[1][1], m.v[2][1], 
        m.v[0][2], m.v[1][2], m.v[2][2]}};
   return result;
}

// matrix product for 3x3 matrices
matrix_3x3_t static inline mmul3(const matrix_3x3_t m1, const matrix_3x3_t m2) {
   matrix_3x3_t result= 
   {.v={m1.v[0][0]* m2.v[0][0]+m1.v[0][1]* m2.v[1][0]+m1.v[0][2]* m2.v[2][0], m1.v[0][0]* m2.v[0][1]+m1.v[0][1]* m2.v[1][1]+m1.v[0][2]* m2.v[2][1], m1.v[0][0]* m2.v[0][2]+m1.v[0][1]* m2.v[1][2]+m1.v[0][2]* m2.v[2][2], 
        m1.v[1][0]* m2.v[0][0]+m1.v[1][1]* m2.v[1][0]+m1.v[1][2]* m2.v[2][0], m1.v[1][0]* m2.v[0][1]+m1.v[1][1]* m2.v[1][1]+m1.v[1][2]* m2.v[2][1], m1.v[1][0]* m2.v[0][2]+m1.v[1][1]* m2.v[1][2]+m1.v[1][2]* m2.v[2][2], 
        m1.v[2][0]* m2.v[0][0]+m1.v[2][1]* m2.v[1][0]+m1.v[2][2]* m2.v[2][0], m1.v[2][0]* m2.v[0][1]+m1.v[2][1]* m2.v[1][1]+m1.v[2][2]* m2.v[2][1], m1.v[2][0]* m2.v[0][2]+m1.v[2][1]* m2.v[1][2]+m1.v[2][2]* m2.v[2][2]}};
   return result;
}

// scalar/matrix product for 3x3 matrices
matrix_3x3_t static inline smmul3(const float s, const matrix_3x3_t m) {
   matrix_3x3_t result= 
   {.v={s * m.v[0][0], s * m.v[0][1], s * m.v[0][2], 
        s * m.v[1][0], s * m.v[1][1], s * m.v[1][2], 
        s * m.v[2][0], s * m.v[2][1], s * m.v[2][2]}};
   return result;
}

// matrix/vector product for 3x3 matrices
vector_3_t static inline mvmul3(const matrix_3x3_t m1, const vector_3_t vec) {
   vector_3_t result= 
   {.v={m1.v[0][0]* vec.v[0]+m1.v[0][1]* vec.v[1]+m1.v[0][2]* vec.v[2], 
        m1.v[1][0]* vec.v[0]+m1.v[1][1]* vec.v[1]+m1.v[1][2]* vec.v[2], 
        m1.v[2][0]* vec.v[0]+m1.v[2][1]* vec.v[1]+m1.v[2][2]* vec.v[2]}};
   return result;
}

// scalar/vector product for 3x3 vectors
vector_3_t static inline svmul3(const float s, const vector_3_t vec) {
   vector_3_t result= 
   {.v={s* vec.v[0], 
        s* vec.v[1], 
        s* vec.v[2]}};
   return result;
}

// tensor product for 3D vectors
matrix_3x3_t static inline tp3(const vector_3_t vec1, const vector_3_t vec2) {
   matrix_3x3_t result= 
   {.v={vec1.v[0]* vec2.v[0], vec1.v[0]* vec2.v[1], vec1.v[0]* vec2.v[2], 
        vec1.v[1]* vec2.v[0], vec1.v[1]* vec2.v[1], vec1.v[1]* vec2.v[2], 
        vec1.v[2]* vec2.v[0], vec1.v[2]* vec2.v[1], vec1.v[2]* vec2.v[2]}};
   return result;
}

// scalar product for 3D vectors
float static inline sp3(const vector_3_t vec1, const vector_3_t vec2) {
   return (vec1.v[0]* vec2.v[0]+vec1.v[1]* vec2.v[1]+vec1.v[2]* vec2.v[2]);
 }


// pointwise add for 3x3 matrices
matrix_3x3_t static inline madd3(const matrix_3x3_t m1, const matrix_3x3_t m2) {
   matrix_3x3_t result= 
   {.v={m1.v[0][0]+ m2.v[0][0], m1.v[0][1]+ m2.v[0][1], m1.v[0][2]+ m2.v[0][2], 
        m1.v[1][0]+ m2.v[1][0], m1.v[1][1]+ m2.v[1][1], m1.v[1][2]+ m2.v[1][2], 
        m1.v[2][0]+ m2.v[2][0], m1.v[2][1]+ m2.v[2][1], m1.v[2][2]+ m2.v[2][2]}};
   return result;
}

// pointwise add for 3x3 vectors
vector_3_t static inline vadd3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1], 
        v1.v[2]+ v2.v[2]}};
   return result;
}


// pointwise sub for 3x3 matrices
matrix_3x3_t static inline msub3(const matrix_3x3_t m1, const matrix_3x3_t m2) {
   matrix_3x3_t result= 
   {.v={m1.v[0][0]- m2.v[0][0], m1.v[0][1]- m2.v[0][1], m1.v[0][2]- m2.v[0][2], 
        m1.v[1][0]- m2.v[1][0], m1.v[1][1]- m2.v[1][1], m1.v[1][2]- m2.v[1][2], 
        m1.v[2][0]- m2.v[2][0], m1.v[2][1]- m2.v[2][1], m1.v[2][2]- m2.v[2][2]}};
   return result;
}

// pointwise sub for 3x3 vectors
vector_3_t static inline vsub3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1], 
        v1.v[2]- v2.v[2]}};
   return result;
}


// pointwise pwmul for 3x3 matrices
matrix_3x3_t static inline mpwmul3(const matrix_3x3_t m1, const matrix_3x3_t m2) {
   matrix_3x3_t result= 
   {.v={m1.v[0][0]* m2.v[0][0], m1.v[0][1]* m2.v[0][1], m1.v[0][2]* m2.v[0][2], 
        m1.v[1][0]* m2.v[1][0], m1.v[1][1]* m2.v[1][1], m1.v[1][2]* m2.v[1][2], 
        m1.v[2][0]* m2.v[2][0], m1.v[2][1]* m2.v[2][1], m1.v[2][2]* m2.v[2][2]}};
   return result;
}

// pointwise pwmul for 3x3 vectors
vector_3_t static inline vpwmul3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1], 
        v1.v[2]* v2.v[2]}};
   return result;
}


//squared frobenius norm  for 3x3 matrices
float static inline sqr_f_norm3(const matrix_3x3_t m) {
   float result= 
   m.v[0][0]* m.v[0][0] + m.v[0][1]* m.v[0][1] + m.v[0][2]* m.v[0][2] +
   m.v[1][0]* m.v[1][0] + m.v[1][1]* m.v[1][1] + m.v[1][2]* m.v[1][2] +
   m.v[2][0]* m.v[2][0] + m.v[2][1]* m.v[2][1] + m.v[2][2]* m.v[2][2];
   return result;
}

//sum of each row  for 3x3 matrices
vector_3_t static inline row_sum3(const matrix_3x3_t m) {
   vector_3_t result= {.v={
   m.v[0][0] + m.v[0][1] + m.v[0][2] , 
   m.v[1][0] + m.v[1][1] + m.v[1][2] , 
   m.v[2][0] + m.v[2][1] + m.v[2][2]}};
   return result;
}

//sum of each column  for 3x3 matrices
vector_3_t static inline col_sum3(const matrix_3x3_t m) {
   vector_3_t result= {.v={
   m.v[0][0] + m.v[1][0] + m.v[2][0] , 
   m.v[0][1] + m.v[1][1] + m.v[2][1] , 
   m.v[0][2] + m.v[1][2] + m.v[2][2]}};
   return result;
}

//sum for 3D vectors
float static inline sum3(const vector_3_t vec) {
   float result= vec.v[0] + vec.v[1] + vec.v[2];
   return result;
}

//trace for 3D matrices
float static inline trace3(const matrix_3x3_t m) {
   return sum3(diag_vector3(m));
}

//squared norm  for 3D vectors
float static inline sqr_norm3(const vector_3_t vec) {
   float result= vec.v[0]* vec.v[0] + vec.v[1]* vec.v[1] + vec.v[2]* vec.v[2];
   return result;
}


typedef struct matrix_4x4_t {
   float v[4][4];
} matrix_4x4_t; 


typedef struct vector_4_t {
   float v[4];
} vector_4_t;

static const matrix_4x4_t zero_4x4= 
   {.v={{0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}} };

static const matrix_4x4_t ones_4x4= 
   {.v={{1.0f, 1.0f, 1.0f, 1.0f}, 
        {1.0f, 1.0f, 1.0f, 1.0f}, 
        {1.0f, 1.0f, 1.0f, 1.0f}, 
        {1.0f, 1.0f, 1.0f, 1.0f}} };

static const matrix_4x4_t ident_4x4= 
   {.v={{1.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 1.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 1.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 1.0f}} };

// create diagonal matrix
matrix_4x4_t static inline diag_4x4(const vector_4_t v) {
   matrix_4x4_t result= zero_4x4;
   result.v[0][0]=v.v[0];result.v[1][1]=v.v[1];result.v[2][2]=v.v[2];result.v[3][3]=v.v[3];   return result;
}

// return vector of a row
vector_4_t static inline row4(const matrix_4x4_t m, int row) {
   vector_4_t  result= {.v={m.v[row][0], m.v[row][1], m.v[row][2], m.v[row][3]}};
   return result;
}

// return vector of a column
vector_4_t static inline col4(const matrix_4x4_t m, int col) {
   vector_4_t  result= {.v={m.v[0][col], m.v[1][col], m.v[2][col], m.v[3][col]}};
   return result;
}

// return matrix diagonal as a vector
vector_4_t static inline diag_vector4(const matrix_4x4_t m) {
   vector_4_t result= {.v={m.v[0][0], m.v[1][1], m.v[2][2], m.v[3][3]}};
   return result;
}

// transpose of a matrix
matrix_4x4_t static inline trans4(const matrix_4x4_t m) {
   matrix_4x4_t result= 
   {.v={m.v[0][0], m.v[1][0], m.v[2][0], m.v[3][0], 
        m.v[0][1], m.v[1][1], m.v[2][1], m.v[3][1], 
        m.v[0][2], m.v[1][2], m.v[2][2], m.v[3][2], 
        m.v[0][3], m.v[1][3], m.v[2][3], m.v[3][3]}};
   return result;
}

// matrix product for 4x4 matrices
matrix_4x4_t static inline mmul4(const matrix_4x4_t m1, const matrix_4x4_t m2) {
   matrix_4x4_t result= 
   {.v={m1.v[0][0]* m2.v[0][0]+m1.v[0][1]* m2.v[1][0]+m1.v[0][2]* m2.v[2][0]+m1.v[0][3]* m2.v[3][0], m1.v[0][0]* m2.v[0][1]+m1.v[0][1]* m2.v[1][1]+m1.v[0][2]* m2.v[2][1]+m1.v[0][3]* m2.v[3][1], m1.v[0][0]* m2.v[0][2]+m1.v[0][1]* m2.v[1][2]+m1.v[0][2]* m2.v[2][2]+m1.v[0][3]* m2.v[3][2], m1.v[0][0]* m2.v[0][3]+m1.v[0][1]* m2.v[1][3]+m1.v[0][2]* m2.v[2][3]+m1.v[0][3]* m2.v[3][3], 
        m1.v[1][0]* m2.v[0][0]+m1.v[1][1]* m2.v[1][0]+m1.v[1][2]* m2.v[2][0]+m1.v[1][3]* m2.v[3][0], m1.v[1][0]* m2.v[0][1]+m1.v[1][1]* m2.v[1][1]+m1.v[1][2]* m2.v[2][1]+m1.v[1][3]* m2.v[3][1], m1.v[1][0]* m2.v[0][2]+m1.v[1][1]* m2.v[1][2]+m1.v[1][2]* m2.v[2][2]+m1.v[1][3]* m2.v[3][2], m1.v[1][0]* m2.v[0][3]+m1.v[1][1]* m2.v[1][3]+m1.v[1][2]* m2.v[2][3]+m1.v[1][3]* m2.v[3][3], 
        m1.v[2][0]* m2.v[0][0]+m1.v[2][1]* m2.v[1][0]+m1.v[2][2]* m2.v[2][0]+m1.v[2][3]* m2.v[3][0], m1.v[2][0]* m2.v[0][1]+m1.v[2][1]* m2.v[1][1]+m1.v[2][2]* m2.v[2][1]+m1.v[2][3]* m2.v[3][1], m1.v[2][0]* m2.v[0][2]+m1.v[2][1]* m2.v[1][2]+m1.v[2][2]* m2.v[2][2]+m1.v[2][3]* m2.v[3][2], m1.v[2][0]* m2.v[0][3]+m1.v[2][1]* m2.v[1][3]+m1.v[2][2]* m2.v[2][3]+m1.v[2][3]* m2.v[3][3], 
        m1.v[3][0]* m2.v[0][0]+m1.v[3][1]* m2.v[1][0]+m1.v[3][2]* m2.v[2][0]+m1.v[3][3]* m2.v[3][0], m1.v[3][0]* m2.v[0][1]+m1.v[3][1]* m2.v[1][1]+m1.v[3][2]* m2.v[2][1]+m1.v[3][3]* m2.v[3][1], m1.v[3][0]* m2.v[0][2]+m1.v[3][1]* m2.v[1][2]+m1.v[3][2]* m2.v[2][2]+m1.v[3][3]* m2.v[3][2], m1.v[3][0]* m2.v[0][3]+m1.v[3][1]* m2.v[1][3]+m1.v[3][2]* m2.v[2][3]+m1.v[3][3]* m2.v[3][3]}};
   return result;
}

// scalar/matrix product for 4x4 matrices
matrix_4x4_t static inline smmul4(const float s, const matrix_4x4_t m) {
   matrix_4x4_t result= 
   {.v={s * m.v[0][0], s * m.v[0][1], s * m.v[0][2], s * m.v[0][3], 
        s * m.v[1][0], s * m.v[1][1], s * m.v[1][2], s * m.v[1][3], 
        s * m.v[2][0], s * m.v[2][1], s * m.v[2][2], s * m.v[2][3], 
        s * m.v[3][0], s * m.v[3][1], s * m.v[3][2], s * m.v[3][3]}};
   return result;
}

// matrix/vector product for 4x4 matrices
vector_4_t static inline mvmul4(const matrix_4x4_t m1, const vector_4_t vec) {
   vector_4_t result= 
   {.v={m1.v[0][0]* vec.v[0]+m1.v[0][1]* vec.v[1]+m1.v[0][2]* vec.v[2]+m1.v[0][3]* vec.v[3], 
        m1.v[1][0]* vec.v[0]+m1.v[1][1]* vec.v[1]+m1.v[1][2]* vec.v[2]+m1.v[1][3]* vec.v[3], 
        m1.v[2][0]* vec.v[0]+m1.v[2][1]* vec.v[1]+m1.v[2][2]* vec.v[2]+m1.v[2][3]* vec.v[3], 
        m1.v[3][0]* vec.v[0]+m1.v[3][1]* vec.v[1]+m1.v[3][2]* vec.v[2]+m1.v[3][3]* vec.v[3]}};
   return result;
}

// scalar/vector product for 4x4 vectors
vector_4_t static inline svmul4(const float s, const vector_4_t vec) {
   vector_4_t result= 
   {.v={s* vec.v[0], 
        s* vec.v[1], 
        s* vec.v[2], 
        s* vec.v[3]}};
   return result;
}

// tensor product for 4D vectors
matrix_4x4_t static inline tp4(const vector_4_t vec1, const vector_4_t vec2) {
   matrix_4x4_t result= 
   {.v={vec1.v[0]* vec2.v[0], vec1.v[0]* vec2.v[1], vec1.v[0]* vec2.v[2], vec1.v[0]* vec2.v[3], 
        vec1.v[1]* vec2.v[0], vec1.v[1]* vec2.v[1], vec1.v[1]* vec2.v[2], vec1.v[1]* vec2.v[3], 
        vec1.v[2]* vec2.v[0], vec1.v[2]* vec2.v[1], vec1.v[2]* vec2.v[2], vec1.v[2]* vec2.v[3], 
        vec1.v[3]* vec2.v[0], vec1.v[3]* vec2.v[1], vec1.v[3]* vec2.v[2], vec1.v[3]* vec2.v[3]}};
   return result;
}

// scalar product for 4D vectors
float static inline sp4(const vector_4_t vec1, const vector_4_t vec2) {
   return (vec1.v[0]* vec2.v[0]+vec1.v[1]* vec2.v[1]+vec1.v[2]* vec2.v[2]+vec1.v[3]* vec2.v[3]);
 }


// pointwise add for 4x4 matrices
matrix_4x4_t static inline madd4(const matrix_4x4_t m1, const matrix_4x4_t m2) {
   matrix_4x4_t result= 
   {.v={m1.v[0][0]+ m2.v[0][0], m1.v[0][1]+ m2.v[0][1], m1.v[0][2]+ m2.v[0][2], m1.v[0][3]+ m2.v[0][3], 
        m1.v[1][0]+ m2.v[1][0], m1.v[1][1]+ m2.v[1][1], m1.v[1][2]+ m2.v[1][2], m1.v[1][3]+ m2.v[1][3], 
        m1.v[2][0]+ m2.v[2][0], m1.v[2][1]+ m2.v[2][1], m1.v[2][2]+ m2.v[2][2], m1.v[2][3]+ m2.v[2][3], 
        m1.v[3][0]+ m2.v[3][0], m1.v[3][1]+ m2.v[3][1], m1.v[3][2]+ m2.v[3][2], m1.v[3][3]+ m2.v[3][3]}};
   return result;
}

// pointwise add for 4x4 vectors
vector_4_t static inline vadd4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1], 
        v1.v[2]+ v2.v[2], 
        v1.v[3]+ v2.v[3]}};
   return result;
}


// pointwise sub for 4x4 matrices
matrix_4x4_t static inline msub4(const matrix_4x4_t m1, const matrix_4x4_t m2) {
   matrix_4x4_t result= 
   {.v={m1.v[0][0]- m2.v[0][0], m1.v[0][1]- m2.v[0][1], m1.v[0][2]- m2.v[0][2], m1.v[0][3]- m2.v[0][3], 
        m1.v[1][0]- m2.v[1][0], m1.v[1][1]- m2.v[1][1], m1.v[1][2]- m2.v[1][2], m1.v[1][3]- m2.v[1][3], 
        m1.v[2][0]- m2.v[2][0], m1.v[2][1]- m2.v[2][1], m1.v[2][2]- m2.v[2][2], m1.v[2][3]- m2.v[2][3], 
        m1.v[3][0]- m2.v[3][0], m1.v[3][1]- m2.v[3][1], m1.v[3][2]- m2.v[3][2], m1.v[3][3]- m2.v[3][3]}};
   return result;
}

// pointwise sub for 4x4 vectors
vector_4_t static inline vsub4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1], 
        v1.v[2]- v2.v[2], 
        v1.v[3]- v2.v[3]}};
   return result;
}


// pointwise pwmul for 4x4 matrices
matrix_4x4_t static inline mpwmul4(const matrix_4x4_t m1, const matrix_4x4_t m2) {
   matrix_4x4_t result= 
   {.v={m1.v[0][0]* m2.v[0][0], m1.v[0][1]* m2.v[0][1], m1.v[0][2]* m2.v[0][2], m1.v[0][3]* m2.v[0][3], 
        m1.v[1][0]* m2.v[1][0], m1.v[1][1]* m2.v[1][1], m1.v[1][2]* m2.v[1][2], m1.v[1][3]* m2.v[1][3], 
        m1.v[2][0]* m2.v[2][0], m1.v[2][1]* m2.v[2][1], m1.v[2][2]* m2.v[2][2], m1.v[2][3]* m2.v[2][3], 
        m1.v[3][0]* m2.v[3][0], m1.v[3][1]* m2.v[3][1], m1.v[3][2]* m2.v[3][2], m1.v[3][3]* m2.v[3][3]}};
   return result;
}

// pointwise pwmul for 4x4 vectors
vector_4_t static inline vpwmul4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1], 
        v1.v[2]* v2.v[2], 
        v1.v[3]* v2.v[3]}};
   return result;
}


//squared frobenius norm  for 4x4 matrices
float static inline sqr_f_norm4(const matrix_4x4_t m) {
   float result= 
   m.v[0][0]* m.v[0][0] + m.v[0][1]* m.v[0][1] + m.v[0][2]* m.v[0][2] + m.v[0][3]* m.v[0][3] +
   m.v[1][0]* m.v[1][0] + m.v[1][1]* m.v[1][1] + m.v[1][2]* m.v[1][2] + m.v[1][3]* m.v[1][3] +
   m.v[2][0]* m.v[2][0] + m.v[2][1]* m.v[2][1] + m.v[2][2]* m.v[2][2] + m.v[2][3]* m.v[2][3] +
   m.v[3][0]* m.v[3][0] + m.v[3][1]* m.v[3][1] + m.v[3][2]* m.v[3][2] + m.v[3][3]* m.v[3][3];
   return result;
}

//sum of each row  for 4x4 matrices
vector_4_t static inline row_sum4(const matrix_4x4_t m) {
   vector_4_t result= {.v={
   m.v[0][0] + m.v[0][1] + m.v[0][2] + m.v[0][3] , 
   m.v[1][0] + m.v[1][1] + m.v[1][2] + m.v[1][3] , 
   m.v[2][0] + m.v[2][1] + m.v[2][2] + m.v[2][3] , 
   m.v[3][0] + m.v[3][1] + m.v[3][2] + m.v[3][3]}};
   return result;
}

//sum of each column  for 4x4 matrices
vector_4_t static inline col_sum4(const matrix_4x4_t m) {
   vector_4_t result= {.v={
   m.v[0][0] + m.v[1][0] + m.v[2][0] + m.v[3][0] , 
   m.v[0][1] + m.v[1][1] + m.v[2][1] + m.v[3][1] , 
   m.v[0][2] + m.v[1][2] + m.v[2][2] + m.v[3][2] , 
   m.v[0][3] + m.v[1][3] + m.v[2][3] + m.v[3][3]}};
   return result;
}

//sum for 4D vectors
float static inline sum4(const vector_4_t vec) {
   float result= vec.v[0] + vec.v[1] + vec.v[2] + vec.v[3];
   return result;
}

//trace for 4D matrices
float static inline trace4(const matrix_4x4_t m) {
   return sum4(diag_vector4(m));
}

//squared norm  for 4D vectors
float static inline sqr_norm4(const vector_4_t vec) {
   float result= vec.v[0]* vec.v[0] + vec.v[1]* vec.v[1] + vec.v[2]* vec.v[2] + vec.v[3]* vec.v[3];
   return result;
}

#ifdef __cplusplus
}
#endif

#endif
