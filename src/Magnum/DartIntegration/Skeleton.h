#ifndef Magnum_DartIntegration_Skeleton_h
#define Magnum_DartIntegration_Skeleton_h
/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018
              Vladimír Vondruš <mosra@centrum.cz>
    Copyright © 2018 Konstantinos Chatzilygeroudis <costashatz@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

/** @file
 * @brief Class @ref Magnum::DartIntegration::Skeleton
 */

#include <functional>
#include <memory>
#include <vector>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include "Magnum/DartIntegration/Object.h"

namespace Magnum { namespace DartIntegration {

/**
@brief DART Physics Skeleton (i.e., Multi-Body entity)

Parses a DART `Skeleton` for easy usage in Magnum. It basically parses the
`Skeleton` and keeps track of a list of @ref Object instances.

@section DartIntegration-Skeleton-usage Usage

Common usage is to create a DART `SkeletonPtr` and then instantiate
a `Skeleton` by passing a parent @ref SceneGraph feature and
the `SkeletonPtr` to its constructor:

@code{.cpp}
dart::dynamics::SkeletonPtr skel = createSkeletonInDart();
SceneGraph::Object<SceneGraph::MatrixTransformation3D> object;
Skeleton* dartSkel = new Skeleton{object, skel};
@endcode

Only the DART Skeleton can affect the transformation of the Magnum
@ref Skeleton and not the other way around. To get the latest DART
transformations, you should update the objects using @ref updateObjects().

@section DartIntegration-Skeleton-limitations Limitations

-   `SoftBodyNode`s are not supported yet
-   When changing the structure of a `SkeletonPtr`, @ref Skeleton will not
    automatically update
*/
class MAGNUM_DARTINTEGRATION_EXPORT Skeleton {
    public:
         /**
         * @brief Constructor
         * @param parent    Parent object
         * @param skeleton  DART `SkeletonPtr` to parse
         */
        template<class T> explicit Skeleton(T& parent, dart::dynamics::SkeletonPtr skeleton = nullptr) {
            if(skeleton) parseSkeleton(parent, skeleton);
        }

        /**
         * @brief Parse a DART skeleton
         * @param parent    Parent object
         * @param skeleton  DART `SkeletonPtr` to parse
         * @return Reference to self
         */
        template<class T> Skeleton& parseSkeleton(T& parent, dart::dynamics::SkeletonPtr skeleton) {
            /** @todo Support more than one kinematic tree */
            if(skeleton) {
                _objects.clear();
                parseBodyNodeRecursive(parent, *skeleton->getRootBodyNode());
            }

            return *this;
        }

        /** @brief Update all child objects */
        Skeleton& updateObjects();

        /** @brief Get all objects */
        std::vector<std::reference_wrapper<Object>> objects();

        /** @brief Get all objects associated with `ShapeNode`s */
        std::vector<std::reference_wrapper<Object>> shapeObjects();

        /** @brief Get all objects associated with `BodyNode`s */
        std::vector<std::reference_wrapper<Object>> bodyObjects();

    private:
        template<class T> void parseBodyNodeRecursive(T& parent, dart::dynamics::BodyNode& bn);

        std::vector<std::unique_ptr<Object>> _objects;
};

template<class T> void Skeleton::parseBodyNodeRecursive(T& parent, dart::dynamics::BodyNode& bn) {
    /* Parse the BodyNode -- we care only about visuals */
    auto visualShapes = bn.getShapeNodesWith<dart::dynamics::VisualAspect>();

    /* Create an object of the BodyNode to keep track of transformations */
    auto object = new T{&parent};
    _objects.push_back(std::unique_ptr<Object>(new Object{*object, &bn}));
    for(auto& shape: visualShapes) {
        /* create objects for the ShapeNodes to keep track of inner transformations */
        auto shapeObj = new T{object};
        _objects.emplace_back(std::unique_ptr<Object>(new Object{*shapeObj, shape}));
    }

    /* parse the children recursively */
    std::size_t numChilds = bn.getNumChildBodyNodes();
    for(std::size_t i = 0; i < numChilds; i++) {
        /* pass as parent the newly created object */
        parseBodyNodeRecursive(*object, *bn.getChildBodyNode(i));
    }
}

}}

#endif
