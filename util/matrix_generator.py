# /*******************************************************************************
#  * Copyright (c) 2009-2014, MAV'RIC Development Team
#  * All rights reserved.
#  * 
#  * Redistribution and use in source and binary forms, with or without 
#  * modification, are permitted provided that the following conditions are met:
#  * 
#  * 1. Redistributions of source code must retain the above copyright notice, 
#  * this list of conditions and the following disclaimer.
#  * 
#  * 2. Redistributions in binary form must reproduce the above copyright notice, 
#  * this list of conditions and the following disclaimer in the documentation 
#  * and/or other materials provided with the distribution.
#  * 
#  * 3. Neither the name of the copyright holder nor the names of its contributors
#  * may be used to endorse or promote products derived from this software without
#  * specific prior written permission.
#  * 
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
#  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
#  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
#  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#  * POSSIBILITY OF SUCH DAMAGE.
#  ******************************************************************************/

# /*******************************************************************************
#  * \file matrix_generator.py
#  * 
#  * \author MAV'RIC Team
#  * \author Felix Schill
#  *   
#  * \brief Script to generate matrix library
#  *
#  ******************************************************************************/

import sys

def generate_types( dim):
    output ="typedef struct matrix_%ix%i_t {\n"%(dim, dim)+\
                    "   float v[%i][%i];\n"%(dim,  dim)+\
                    "} matrix_%ix%i_t; \n"%(dim, dim)+"\n\n"
    output+="typedef struct vector_%i_t {\n"%dim+\
                    "   float v[%i];\n"%(dim)+\
                    "} vector_%i_t;\n\n"%dim
 
  #define constants    
    type_ext="%ix%i"%(dim,  dim)
    output+="static const matrix_"+type_ext+"_t zero_"+type_ext+"= \n   {.v={{";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="0.0f"
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        {"
    output+="}} };\n\n"

    output+="static const matrix_"+type_ext+"_t ones_"+type_ext+"= \n   {.v={{";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="1.0f"
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        {"
    output+="}} };\n\n"


    output+="static const matrix_"+type_ext+"_t ident_"+type_ext+"= \n   {.v={{";
    for i in range(0, dim):
        for j in range(0, dim):
            if i==j:
                output+="1.0f"
            else:
                output+="0.0f"
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        {"
    output+="}} };\n\n"    

    #define initialisers
    output+="// create diagonal matrix\n"
    output+="matrix_"+type_ext+"_t static inline diag_"+type_ext+"(const vector_%i_t v) {\n"%dim+\
                     "   matrix_"+type_ext+"_t result= zero_"+type_ext+";\n   ";
    for i in range(0, dim):
            output+="result.v[%i][%i]=v.v[%i];"%(i,  i, i)
    output+="   return result;\n}\n\n"
    
    #define getters
    output+="// return vector of a row\n"
    output+="vector_%i_t"%dim+" static inline row%i"%dim+"(const matrix_"+type_ext+"_t m, int32_t row) {\n"+\
                     "   vector_%i_t"%dim+"  result= {.v={";
    for i in range(0, dim):
            output+="m.v[row][%i]"%(  i)
            if i!=dim-1:
                output+=", "
    output+="}};\n"
    output+="   return result;\n}\n\n"

    output+="// return vector of a column\n"
    output+="vector_%i_t"%dim+" static inline col%i"%dim+"(const matrix_"+type_ext+"_t m, int32_t col) {\n"+\
                     "   vector_%i_t"%dim+"  result= {.v={";
    for i in range(0, dim):
            output+="m.v[%i][col]"%(i)
            if i!=dim-1:
                output+=", "
    output+="}};\n"
    output+="   return result;\n}\n\n"


    output+="// return matrix diagonal as a vector\n"
    output+="vector_%i_t"%dim+" static inline diag_vector%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                     "   vector_%i_t"%dim+" result= {.v={";
    for i in range(0, dim):
            output+="m.v[%i][%i]"%(i,  i)
            if i!=dim-1:
                output+=", "
    output+="}};\n"
    output+="   return result;\n}\n\n"


    #define transpose
    output+="// transpose of a matrix\n"    
    output+="matrix_"+type_ext+"_t static inline trans%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                     "   matrix_"+type_ext+"_t result= \n   {.v={";
    for i in range(0, dim):
        output+="{"
        for j in range(0, dim):
            output+="m.v[%i][%i]"%(j,  i)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        "
    output+="}}};\n"
    output+="   return result;\n}\n"

    return output
    
    
def generate_mul(dim):
    type_ext="%ix%i"%(dim,  dim)
    output=""
    output+="// matrix product for "+type_ext+" matrices\n"

    output +="matrix_"+type_ext+"_t static inline mmul%i"%dim+"(const matrix_"+type_ext+"_t m1, const matrix_"+type_ext+"_t m2) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        output+="{"
        for j in range(0, dim):
            for k in range(0, dim):
                output+="m1.v[%i][%i]* m2.v[%i][%i]"%(i,  k,  k,  j)
                if k!=dim-1:
                    output+="+"
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        "
        
    output+="}}};\n"
    output+="   return result;\n}\n\n"
    
    
    output+="// scalar/matrix product for "+type_ext+" matrices\n"

    output +="matrix_"+type_ext+"_t static inline smmul%i"%dim+"(const float s, const matrix_"+type_ext+"_t m) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        output+="{"
        for j in range(0, dim):
            output+="s * m.v[%i][%i]"%(i,  j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        "
        
    output+="}}};\n"
    output+="   return result;\n}\n\n"
    
    #matrix / vector
    output+="// matrix/vector product for "+type_ext+" matrices\n"
    output +="vector_%i_t"%dim + " static inline mvmul%i"%dim+"(const matrix_"+type_ext+"_t m1, const vector_%i_t"%dim + " vec) {\n"+\
                    "   vector_%i_t"%dim + " result= \n   {.v={";
    for i in range(0, dim):
            for k in range(0, dim):
                output+="m1.v[%i][%i]* vec.v[%i]"%(i,  k,  k)
                if k!=dim-1:
                    output+="+"
            if i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
    output+="   return result;\n}\n\n"


    #scalar / vector
    output+="// scalar/vector product for "+type_ext+" vectors\n"
    output +="vector_%i_t"%dim + " static inline svmul%i"%dim+"(const float s, const vector_%i_t"%dim + " vec) {\n"+\
                    "   vector_%i_t"%dim + " result= \n   {.v={";
    for i in range(0, dim):
            output+="s* vec.v[%i]"%(i)
            if i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
    output+="   return result;\n}\n\n"


    #tensor product
    output+="// tensor product for %iD vectors\n"%dim
    output +="matrix_"+type_ext+"_t static inline tp%i"%dim+"(const vector_%i_t"%dim + " vec1, const vector_%i_t"%dim + " vec2) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        output+="{"    
        for j in range(0, dim):
            output+="vec1.v[%i]* vec2.v[%i]"%(i,   j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        "
        
    output+="}}};\n"
    output+="   return result;\n}\n\n"

    #scalar product
    output+="// scalar product for %iD vectors\n"%dim
    output +="float static inline sp%i"%dim+"(const vector_%i_t"%dim + " vec1, const vector_%i_t"%dim + " vec2) {\n"+\
                    "   return (";
    
    for i in range(0, dim):
                output+="vec1.v[%i]* vec2.v[%i]"%(i,  i)
                if i!=dim-1:
                    output+="+"
            
    output+=");\n"
    output+=" }\n\n"

    return output
    
def generate_pointwise(dim,  name,  operator):
    type_ext="%ix%i"%(dim,  dim)
    output="// pointwise "+name+" for "+type_ext+" matrices\n"

    output +="matrix_"+type_ext+"_t static inline m"+name+"%i"%dim+"(const matrix_"+type_ext+"_t m1, const matrix_"+type_ext+"_t m2) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        output+="{"    
        for j in range(0, dim):
            output+="m1.v[%i][%i]"%( i,  j)+operator+" m2.v[%i][%i]"%(i,  j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+="}, \n        "
        
    output+="}}};\n"
    output+="   return result;\n}\n\n"

    output+="// pointwise "+name+" for "+type_ext+" vectors\n"

    output +="vector_%i_t"%dim+" static inline v"+name+"%i"%dim+"(const vector_%i_t"%dim+" v1, const vector_%i_t"%dim+" v2) {\n"+\
                    "   vector_%i_t"%dim+" result= \n   {.v={";
    
    for i in range(0, dim):
            output+="v1.v[%i]"%( i)+operator+" v2.v[%i]"%(i)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
    output+="   return result;\n}\n\n"

    return output
    
    
    
def generate_norms(dim):
    type_ext="%ix%i"%(dim,  dim)
    output="//squared frobenius norm  for "+type_ext+" matrices\n"
    output +="float static inline sqr_f_norm%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                    "   float result= \n   ";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="m.v[%i][%i]"%( i,  j)+"* m.v[%i][%i]"%(i,  j)
            if j!=dim-1:
                output+=" + "
            elif i!=dim-1:
                output+=" +\n   "
            else:
                output+=";\n"
    output+="   return result;\n}\n\n"

    output+="//sum of each row  for "+type_ext+" matrices\n"
    output +="vector_%i_t"%dim+" static inline row_sum%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                    "   vector_%i_t"%dim+" result= {.v={\n   ";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="m.v[%i][%i]"%( i,  j)
            if j!=dim-1:
                output+=" + "
            elif i!=dim-1:
                output+=" , \n   "
    output+="}};\n"
    output+="   return result;\n}\n\n"

    output+="//sum of each column  for "+type_ext+" matrices\n"
    output +="vector_%i_t"%dim+" static inline col_sum%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                    "   vector_%i_t"%dim+" result= {.v={\n   ";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="m.v[%i][%i]"%( j,  i)
            if j!=dim-1:
                output+=" + "
            elif i!=dim-1:
                output+=" , \n   "
    output+="}};\n"
    output+="   return result;\n}\n\n"

    output+="//sum for %iD"%dim+" vectors\n"
    output +="float static inline sum%i"%dim+"(const vector_%i_t vec) {\n"%dim+\
                    "   float result= ";
    for i in range(0, dim):
            output+="vec.v[%i]"%( i)
            if j!=dim-1:
                output+=" + "
            if i!=dim-1:
                output+=" + "
    output+=";\n"
    output+="   return result;\n}\n\n"

    output+="//trace for %iD"%dim+" matrices\n"
    output +="float static inline trace%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"
    output +="   return sum%i(diag_vector%i(m));\n}\n\n"%(dim,  dim)

    output+="//squared norm  for %iD"%dim+" vectors\n"
    output +="float static inline sqr_norm%i"%dim+"(const vector_%i_t vec) {\n"%dim+\
                    "   float result= ";
    for i in range(0, dim):
            output+="vec.v[%i]"%( i)+"* vec.v[%i]"%(i)
            if j!=dim-1:
                output+=" + "
            if i!=dim-1:
                output+=" + "
    output+=";\n"
    output+="   return result;\n}\n\n"
    return output

object = open("small_matrix.h", "w+",0);
sys.stdout = object
#print "#ifndef SMALL_MATRIX_H_\n#define SMALL_MATRIX_H_\n\n\n"
object.write("#ifndef SMALL_MATRIX_H_\n#define SMALL_MATRIX_H_\n\n\n")
object.write('#include"#include <stdbool.h>"\n\n')
for d in range(1,7):
    #print generate_types(d)
    #print generate_mul(d)
    #print generate_pointwise(d,  "add",  "+")
    #print generate_pointwise(d,  "sub",  "-")
    #print generate_pointwise(d,  "pwmul",  "*")
    #print generate_norms(d)
    object.write(generate_types(d))
    object.write(generate_mul(d))
    object.write(generate_pointwise(d,  "add",  "+"))
    object.write(generate_pointwise(d,  "sub",  "-"))
    object.write(generate_pointwise(d,  "pwmul",  "*"))
    object.write(generate_norms(d))
#print "#endif"
object.write("#endif")
object.close()