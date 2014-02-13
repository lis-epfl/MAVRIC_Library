#ifndef SMALL_MATRIX_H_
#define SMALL_MATRIX_H_



typedef struct matrix_2x2_t {
   float v[2][2];
} matrix_2x2_t; 


typedef struct vector_2_t {
   float v[2];
} vector_2_t;

static const matrix_2x2_t zero_2x2= 
   {.v={{0.0f, 0.0f}, 
        {0.0f, 0.0f}} };

static const matrix_2x2_t ident_2x2= 
   {.v={{1.0f, 0.0f}, 
        {0.0f, 1.0f}} };

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


// pointwise add for 2x2 vectors
vector_2_t static inline madd2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1]}};
   return result;
}


// pointwise sub for 2x2 vectors
vector_2_t static inline msub2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1]}};
   return result;
}


// pointwise pwmul for 2x2 vectors
vector_2_t static inline mpwmul2(const vector_2_t v1, const vector_2_t v2) {
   vector_2_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1]}};
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

static const matrix_3x3_t ident_3x3= 
   {.v={{1.0f, 0.0f, 0.0f}, 
        {0.0f, 1.0f, 0.0f}, 
        {0.0f, 0.0f, 1.0f}} };

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


// pointwise add for 3x3 vectors
vector_3_t static inline madd3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1], 
        v1.v[2]+ v2.v[2]}};
   return result;
}


// pointwise sub for 3x3 vectors
vector_3_t static inline msub3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1], 
        v1.v[2]- v2.v[2]}};
   return result;
}


// pointwise pwmul for 3x3 vectors
vector_3_t static inline mpwmul3(const vector_3_t v1, const vector_3_t v2) {
   vector_3_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1], 
        v1.v[2]* v2.v[2]}};
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

static const matrix_4x4_t ident_4x4= 
   {.v={{1.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 1.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 1.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 1.0f}} };

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


// pointwise add for 4x4 vectors
vector_4_t static inline madd4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]+ v2.v[0], 
        v1.v[1]+ v2.v[1], 
        v1.v[2]+ v2.v[2], 
        v1.v[3]+ v2.v[3]}};
   return result;
}


// pointwise sub for 4x4 vectors
vector_4_t static inline msub4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]- v2.v[0], 
        v1.v[1]- v2.v[1], 
        v1.v[2]- v2.v[2], 
        v1.v[3]- v2.v[3]}};
   return result;
}


// pointwise pwmul for 4x4 vectors
vector_4_t static inline mpwmul4(const vector_4_t v1, const vector_4_t v2) {
   vector_4_t result= 
   {.v={v1.v[0]* v2.v[0], 
        v1.v[1]* v2.v[1], 
        v1.v[2]* v2.v[2], 
        v1.v[3]* v2.v[3]}};
   return result;
}


#endif
