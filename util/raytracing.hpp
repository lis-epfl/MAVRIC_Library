/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file raytracing.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Raytracing. Implements rays and intersection with objects (plane, sphere, cylinder... )
 *
 ******************************************************************************/


#ifndef RAYTRACING_HPP_
#define RAYTRACING_HPP_

#include "util/matrix.hpp"
#include <vector>

namespace raytracing
{


//##################################################################################################
// Ray class
//##################################################################################################
/**
 * \brief   Ray
 */
class Ray
{
public:
    /**
     * \brief   Constructor
     *
     * \param   origin      Ray origin
     * \param   direction   Ray direction (unit vector)
     */
    Ray(Vector3f origin = Vector3f{0.0f, 0.0f, 0.0f}, Vector3f direction = Vector3f{1.0f, 0.0f, 0.0f});

    /**
     * \brief   Return origin point
     */
    const Vector3f& origin(void) const;

    /**
     * \brief   Update origin point
     */
    bool set_origin(Vector3f origin);

    /**
     * \brief   Return direction vector
     */
    const Vector3f& direction(void) const;

    /**
     * \brief   Update direction
     */
    bool set_direction(Vector3f origin);

private:
    Vector3f origin_;           ///< Ray origin
    Vector3f direction_;        ///< Ray
};


//##################################################################################################
// Intersection class
//##################################################################################################
/**
 * \brief Intersection betwen a ray and an object
 */
class Intersection
{
public:
    /**
     * \brief   Constructor
     *
     * \param   point       Intersection point
     * \param   normal      Normal to surface at intersection
     * \param   distance    Distance from intersection point to ray origin
     */
    Intersection(Vector3f point = Vector3f{0.0f, 0.0f, 0.0f}, Vector3f normal = Vector3f{1.0f, 0.0f, 0.0f}, float distance = 0.0f);

    /**
     * \brief   Return intersection point
     */
    const Vector3f& point(void) const;

    /**
     * \brief   Update intersection point
     */
    bool set_point(Vector3f point);

    /**
     * \brief   Return normal vector
     */
    const Vector3f& normal(void) const;

    /**
     * \brief   Update normal
     */
    bool set_normal(Vector3f normal);

    /**
     * \brief   Return distance
     */
    float distance(void) const;

    /**
     * \brief   Update normal
     */
    bool set_distance(float distance);

private:
      Vector3f point_;     ///< Intersection point
      Vector3f normal_;    ///< Normal to surface at intersection
      float distance_;     ///< Distance from intersection point to ray origin
};


//##################################################################################################
// Object interface
//##################################################################################################
/**
 * \brief   Object interface
 */
class Object
{
public:
    /**
     * \brief Perform intersection between a ray and the current object
     *
     * \param   ray           Ray to intersect with (input)
     * \param   intersection  Intersection point (output)
     *
     * \return  boolean indicating if an intersection was found
     */
    virtual bool intersect(const Ray& ray, Intersection& intersection) = 0;
};

//##################################################################################################
// World class
//##################################################################################################
/**
 * \brief  World
 */
class World
{
public:
    /**
     * \brief   Add an object to the world
     *
     * \param   obj     Object
     *
     * \return  success
     */
    bool add_object(Object* obj);

    /**
     * \brief   Return intersection between ray and closest object in world
     *
     * \param   ray           Ray to intersect with (input)
     * \param   intersection  Intersection point (output)
     * \param   object        Intersecting object (output)
     *
     * \return  boolean indicating if an intersection was found
     */
    bool intersect(const Ray& ray, Intersection& intersection, Object* object);

private:
    std::vector<Object*>  objects_;
};


//##################################################################################################
// Plane class
//##################################################################################################
/**
 * \brief   Plane
 */
class Plane: public Object
{
public:
    /**
     * \brief   Constructor
     *
     * \param   center      Center of sphere
     * \param   radius      Radius of sphere
     */
    Plane(Vector3f center = Vector3f{0.0f, 0.0f, 0.0f}, Vector3f normal = Vector3f{1.0f, 0.0f, 0.0f});

    /**
     * \brief   Return origin point
     */
    const Vector3f& center(void) const;

    /**
     * \brief   Update center point
     */
    bool set_center(Vector3f center);

    /**
     * \brief   Return origin point
     */
    const Vector3f& normal(void) const;

    /**
     * \brief   Update normal point
     */
    bool set_normal(Vector3f normal);

    /**
     * \brief Perform intersection between a ray and the current object
     *
     * \param   ray           Ray to intersect with (input)
     * \param   intersection  Intersection point (output)
     *
     * \return  boolean indicating if an intersection was found
     */
    bool intersect(const Ray& ray, Intersection& intersection);

private:
    Vector3f center_;       ///< Center of plane
    Vector3f normal_;       ///< Normal to plane
};


//##################################################################################################
// Sphere class
//##################################################################################################
/**
 * \brief   Sphere
 */
class Sphere: public Object
{
public:
    /**
     * \brief   Constructor
     *
     * \param   center      Center of sphere
     * \param   radius      Radius of sphere
     */
    Sphere(Vector3f center = Vector3f{0.0f, 0.0f, 0.0f}, float radius = 1.0f);

    /**
     * \brief   Return origin point
     */
    const Vector3f& center(void) const;

    /**
     * \brief   Update center point
     */
    bool set_center(Vector3f center);

    /**
     * \brief   Return radius vector
     */
    float radius(void) const;

    /**
     * \brief   Update radius
     */
    bool set_radius(float radius);

    /**
     * \brief Perform intersection between a ray and the current object
     *
     * \param   ray           Ray to intersect with (input)
     * \param   intersection  Intersection point (output)
     *
     * \return  boolean indicating if an intersection was found
     */
    bool intersect(const Ray& ray, Intersection& intersection);

private:
    Vector3f center_;       ///< Center of sphere
    float radius_;          ///< Radius of sphere
};


//##################################################################################################
// Cylinder class
//##################################################################################################
/**
 * \brief   Cylinder
 */
class Cylinder: public Object
{
public:
    /**
     * \brief   Constructor
     *
     * \param   center      Center of cylinder
     * \param   axis        Axis of the cylinder (normal vetor)
     * \param   radius      Radius of cylinder
     */
    Cylinder(Vector3f center = Vector3f{0.0f, 0.0f, 0.0f}, Vector3f axis = Vector3f{0.0f, 0.0f, 1.0f}, float radius = 1.0f);

    /**
     * \brief   Return center point
     */
    const Vector3f& center(void) const;

    /**
     * \brief   Update center point
     */
    bool set_center(Vector3f center);

    /**
     * \brief   Return axis
     */
    const Vector3f& axis(void) const;

    /**
     * \brief   Update axis
     */
    bool set_axis(Vector3f axis);

    /**
     * \brief   Return radius vector
     */
    float radius(void) const;

    /**
     * \brief   Update radius
     */
    bool set_radius(float radius);

    /**
     * \brief Perform intersection between a ray and the current object
     *
     * \param   ray           Ray to intersect with (input)
     * \param   intersection  Intersection point (output)
     *
     * \return  boolean indicating if an intersection was found
     */
    bool intersect(const Ray& ray, Intersection& intersection);

private:
    Vector3f center_;       ///< Center of sphere
    Vector3f axis_;         ///< Axis of sphere
    float radius_;          ///< Radius of sphere
    // float height_;          ///< Height of cylinder
};

}

#endif /* RAYTRACING_HPP_ */
