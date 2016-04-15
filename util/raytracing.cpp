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

#include "util/raytracing.hpp"

namespace raytracing
{

extern "C"
{
#include "util/maths.h"
}

float norm(Vector3f& vector)
{
    return maths_fast_sqrt( vector[0]*vector[0]
                          + vector[1]*vector[1]
                          + vector[2]*vector[2] );
}

float dot(const Vector3f& v1, const Vector3f& v2)
{
    return (v1.transpose() % v2)[0];
}


void cross(const Vector3f& v1, const Vector3f& v2, Vector3f& v)
{
    v[0] = v1[1] * v2[2] - v1[2] * v2[1];
    v[1] = v1[2] * v2[0] - v1[0] * v2[2];
    v[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

//##################################################################################################
// Ray class
//##################################################################################################

Ray::Ray(Vector3f origin, Vector3f direction):
    origin_(origin)
{
    // Normalize direction
    set_direction(direction);
}

const Vector3f& Ray::origin(void) const
{
    return origin_;
}

bool Ray::set_origin(Vector3f origin)
{
    origin_ = origin;

    return true;
}

const Vector3f& Ray::direction(void) const
{
    return direction_;
}

bool Ray::set_direction(Vector3f direction)
{
    bool success = false;
    float magnitude = norm(direction);

    if (magnitude != 0.0f)
    {
        direction_[0] = direction[0] / magnitude;
        direction_[1] = direction[1] / magnitude;
        direction_[2] = direction[2] / magnitude;
        success = true;
    }
    else
    {
        success = false;
    }

    return success;
}


//##################################################################################################
// Intersection class
//##################################################################################################
Intersection::Intersection(Vector3f point, Vector3f normal, float distance):
    point_(point),
    distance_(distance)
{
    set_normal(normal);
}

const Vector3f& Intersection::point(void) const
{
    return point_;
}


bool Intersection::set_point(Vector3f point)
{
    point_ = point;
    return true;
}


const Vector3f& Intersection::normal(void) const
{
    return normal_;
}

bool Intersection::set_normal(Vector3f normal)
{
    bool success = false;
    float magnitude = norm(normal);

    if (magnitude != 0.0f)
    {
        normal_[0] = normal[0] / magnitude;
        normal_[1] = normal[1] / magnitude;
        normal_[2] = normal[2] / magnitude;
        success = true;
    }
    else
    {
        success = false;
    }

    return success;
}


float Intersection::distance(void) const
{
    return distance_;
}

bool Intersection::set_distance(float distance)
{
    distance_ = maths_f_abs(distance);
    return true;
}


//##################################################################################################
// World class
//##################################################################################################
bool World::add_object(Object* obj)
{
    bool success = false;

    if (obj != NULL)
    {
        success = true;
        objects_.push_back(obj);
    }
    else
    {
        success = false;
    }

    return success;
}

bool World::intersect(const Ray& ray, Intersection& intersection, Object* object)
{
    bool success = false;
    Intersection inter_tmp;
    uint32_t object_count = objects_.size();

    intersection.set_distance(1000.0f);

    for (uint32_t i = 0; i < object_count; i++)
    {
        if (objects_[i]->intersect(ray, inter_tmp))
        {
            success = true;
            if (inter_tmp.distance() < intersection.distance())
            {
                // This intersection is the best
                intersection = inter_tmp;
            }
        }
    }

    return success;
}


//##################################################################################################
// Plane class
//##################################################################################################
Plane::Plane(Vector3f center, Vector3f normal):
  center_(center)
{
    set_normal(normal);
}

const Vector3f& Plane::center(void) const
{
    return center_;
}


bool Plane::set_center(Vector3f center)
{
    center_ = center;

    return true;
}


const Vector3f& Plane::normal(void) const
{
    return normal_;
}


bool Plane::set_normal(Vector3f normal)
{
    bool success = false;
    float magnitude = norm(normal);

    if (magnitude != 0.0f)
    {
        normal_[0] = normal[0] / magnitude;
        normal_[1] = normal[1] / magnitude;
        normal_[2] = normal[2] / magnitude;
        success = true;
    }
    else
    {
        success = false;
    }

    return success;
}


bool Plane::intersect(const Ray& ray, Intersection& intersection)
{
    bool success = false;

    float den = dot(ray.direction(), normal_);

    if (den != 0.0f)
    {
        float d = dot(center_ - ray.origin(), normal_) / den;

        if ( d > 0.0f )
        {
            success = true;
            intersection.set_point(ray.origin() + ray.direction() * d);
            intersection.set_distance(d);
            intersection.set_normal(ray.origin() - intersection.point());
        }
        else
        {
            // No intersection
            success = false;
        }
    }
    else
    {
        // No intersection
        success = false;
    }

    return success;
}


//##################################################################################################
// Sphere class
//##################################################################################################
Sphere::Sphere(Vector3f center, float radius):
  center_(center),
  radius_(radius)
{}

const Vector3f& Sphere::center(void) const
{
    return center_;
}


bool Sphere::set_center(Vector3f center)
{
    center_ = center;

    return true;
}


float Sphere::radius(void) const
{
    return radius_;
}


bool Sphere::set_radius(float radius)
{
    radius_ = maths_f_abs(radius);

    return true;
}


bool Sphere::intersect(const Ray& ray, Intersection& intersection)
{
    bool success = false;

    // Vector CO from  sphere center (C) to ray origin (O)
    Vector3f co = ray.origin() - center_;

    float q = dot(ray.direction(), co) * dot(ray.direction(), co)
              - dot(co, co)
              + radius_ * radius_;

    if (q < 0)
    {
        // No intersection
        success = false;
    }
    else
    {
        float d  = - dot(ray.direction(), co);
        float d1 = d - maths_fast_sqrt(q);
        float d2 = d + maths_fast_sqrt(q);

        if ( (0.0f < d1) && ((d1 < d2) || (d2 < 0.0f)) )
        {
            success = true;
            intersection.set_point(ray.origin() + ray.direction() * d1);
            intersection.set_distance(d1);
            intersection.set_normal(intersection.point() - center_);
        }
        else if ( (0.0f < d2) && ((d2 < d1) || (d1 < 0.0f)) )
        {
            success = true;
            intersection.set_point(ray.origin() + ray.direction() * d2);
            intersection.set_distance(d2);
            intersection.set_normal(intersection.point() - center_);
        }
        else
        {
            // No intersection
            success = false;
        }
    }

    return success;
}


//##################################################################################################
// Cylinder class
//##################################################################################################
Cylinder::Cylinder(Vector3f center, Vector3f axis, float radius):
  center_(center),
  radius_(radius)
{
    set_axis(axis);
}

const Vector3f& Cylinder::center(void) const
{
    return center_;
}


bool Cylinder::set_center(Vector3f center)
{
    center_ = center;

    return true;
}


float Cylinder::radius(void) const
{
    return radius_;
}


bool Cylinder::set_radius(float radius)
{
    radius_ = maths_f_abs(radius);

    return true;
}


const Vector3f& Cylinder::axis(void) const
{
    return axis_;
}


bool Cylinder::set_axis(Vector3f axis)
{
    bool success = false;
    float magnitude = norm(axis);

    if (magnitude != 0.0f)
    {
        axis_[0] = axis[0] / magnitude;
        axis_[1] = axis[1] / magnitude;
        axis_[2] = axis[2] / magnitude;
        success = true;
    }
    else
    {
        success = false;
    }

    return success;
}


bool Cylinder::intersect(const Ray& ray, Intersection& intersection)
{
    bool success = false;

    Vector3f alpha   = axis_ * dot(ray.direction(), axis_);
    Vector3f deltaP  = ray.origin() - center_;
    Vector3f beta    = axis_ * dot(deltaP, axis_);

    float a = dot(ray.direction() - alpha, ray.direction() - alpha);
    float b = 2 * dot(ray.direction() - alpha, deltaP - beta);
    float c = dot(deltaP - beta, deltaP - beta) - radius_*radius_;

    float discriminant = b*b - 4*a*c;

    if (discriminant < 0)
    {
        // No intersection
        success = false;
    }
    else
    {
        discriminant = sqrt(discriminant);
        float d1 = ((-1 * b) + discriminant) / (2 * a);
        float d2 = ((-1 * b) - discriminant) / (2 * a);
        if ((0.0f < d1) && ((d1 < d2) || (d2 < 0.0f)))
        {
              // Use d1
              success = true;
              intersection.set_point(ray.origin() + ray.direction() * d1);
              intersection.set_distance(d1);
              Vector3f cp = intersection.point() - center_;
              intersection.set_normal( cp - axis_ * dot(axis_, cp) );
        }
        else if ((0.0f < d2) && ((d2 < d1) || (d1 < 0.0f)))
        {
              // Use d2
              success = true;
              intersection.set_point(ray.origin() + ray.direction() * d2);
              intersection.set_distance(d2);
              Vector3f cp = intersection.point() - center_;
              intersection.set_normal( cp - axis_ * dot(axis_, cp) );
        }
        else
        {
              // No intersection
              success = false;
        }
    }

    return success;
}

}
