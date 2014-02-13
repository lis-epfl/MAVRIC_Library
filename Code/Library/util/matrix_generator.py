

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
    
    #define transpose
    type_ext="%ix%i"%(dim,  dim)
    output+="matrix_"+type_ext+"_t static inline trans%i"%dim+"(const matrix_"+type_ext+"_t m) {\n"+\
                     "   matrix_"+type_ext+"_t result= \n   {.v={";
    for i in range(0, dim):
        for j in range(0, dim):
            output+="m.v[%i][%i]"%(j,  i)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
    output+="}};\n"
    output+="   return result;\n}\n"

    return output
    
    
def generate_mul(dim):
    type_ext="%ix%i"%(dim,  dim)
    output=""
    output+="// matrix product for "+type_ext+" matrices\n"

    output +="matrix_"+type_ext+"_t static inline mmul%i"%dim+"(const matrix_"+type_ext+"_t m1, const matrix_"+type_ext+"_t m2) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        for j in range(0, dim):
            for k in range(0, dim):
                output+="m1.v[%i][%i]* m2.v[%i][%i]"%(i,  k,  k,  j)
                if k!=dim-1:
                    output+="+"
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
    output+="   return result;\n}\n\n"
    
    
    output+="// scalar/matrix product for "+type_ext+" matrices\n"

    output +="matrix_"+type_ext+"_t static inline smmul%i"%dim+"(const float s, const matrix_"+type_ext+"_t m) {\n"+\
                    "   matrix_"+type_ext+"_t result= \n   {.v={";
    
    for i in range(0, dim):
        for j in range(0, dim):
            output+="s * m.v[%i][%i]"%(i,  j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
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
        for j in range(0, dim):
            output+="vec1.v[%i]* vec2.v[%i]"%(i,   j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
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
        for j in range(0, dim):
            output+="m1.v[%i][%i]"%( i,  j)+operator+" m2.v[%i][%i]"%(i,  j)
            if j!=dim-1:
                output+=", "
            elif i!=dim-1:
                output+=", \n        "
        
    output+="}};\n"
    output+="   return result;\n}\n\n"

    output="// pointwise "+name+" for "+type_ext+" vectors\n"

    output +="vector_%i_t"%dim+" static inline m"+name+"%i"%dim+"(const vector_%i_t"%dim+" v1, const vector_%i_t"%dim+" v2) {\n"+\
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
    
    
print "#ifndef SMALL_MATRIX_H_\n#define SMALL_MATRIX_H_\n\n\n"
for d in range(2, 5):
    print generate_types(d)
    print generate_mul(d)
    print generate_pointwise(d,  "add",  "+")
    print generate_pointwise(d,  "sub",  "-")
    print generate_pointwise(d,  "pwmul",  "*")
print "#endif"
