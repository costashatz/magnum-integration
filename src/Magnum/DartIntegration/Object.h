#ifndef Magnum_DartIntegration_Object_h
#define Magnum_DartIntegration_Object_h
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
 * @brief Class @ref Magnum::DartIntegration::Object, Class @ref Magnum::DartIntegration::ShapeData
 */

#include <memory>

#include <Corrade/Containers/Array.h>

#include <Magnum/SceneGraph/AbstractFeature.h>
#include <Magnum/SceneGraph/AbstractTranslationRotation3D.h>
#include <Magnum/Trade/PhongMaterialData.h>

#include "Magnum/DartIntegration/visibility.h"

namespace dart { namespace dynamics {
    class BodyNode;
    class ShapeNode;
}}

namespace Magnum {

class Mesh;
class Buffer;
template<UnsignedInt dimensions> class Texture;

namespace DartIntegration {

/**
@brief Shape data

@see @ref Object::_convertShapeNode()
*/
struct ShapeData {
    /** @brief Mesh */
    Mesh* mesh;

    /** @brief vertex Buffer */
    Buffer* vertexBuffer;

    /** @brief index Buffer */
    Buffer* indexBuffer;

    /** @brief Material data */
    Trade::PhongMaterialData material;

    /** @brief Textures */
    Containers::Array<Texture<2>*> textures;
};

/**
@brief DART Physics BodyNode or ShapeNode

Encapsulates `BodyNode` or `ShapeNode` as a @ref SceneGraph feature.

@section DartIntegration-Object-usage Usage

Common usage is to create a @ref Object to share transformation with a DART
`BodyNode` or `ShapeNode` by passing a pointer to its constructor:

@code{.cpp}
dart::dynamics::BodyNode* body = getBodyNodeFromDart();
SceneGraph::Object<SceneGraph::MatrixTransformation3D> object;
Object* obj = new Object{&object, body};
@endcode

or

@code{.cpp}
dart::dynamics::ShapeNode* node = getShapeNodeFromDart();
SceneGraph::Object<SceneGraph::MatrixTransformation3D> object;
Object* obj = new Object{&object, node};
@endcode

Only the DART body/node can affect the transformation of the Magnum object and
not the other way around. To get the latest DART transformation, you should
update the object with @ref update().
*/
class MAGNUM_DARTINTEGRATION_EXPORT Object: public SceneGraph::AbstractBasicFeature3D<Float> {
    public:
        /**
         * @brief Constructor
         * @param object    Object this @ref Object belongs to
         * @param node      DART `ShapeNode` to connect with
         */
        template<class T> Object(T& object, dart::dynamics::ShapeNode* node = nullptr): Object{object, object, node, nullptr} {}

        /**
         * @brief Constructor
         * @param object    Object this @ref Object belongs to
         * @param body      DART `BodyNode` to connect with
         */
        template<class T> Object(T& object, dart::dynamics::BodyNode* body = nullptr): Object{object, object, nullptr, body} {}

        /** @brief Get transformation from DART */
        Object& update();

        /** @brief Get is Object was updated */
        bool used();

        /** @brief Clear usage flag (i.e., set it to false) */
        Object& clearUsed();

        /** @brief Get ShapeData */
        std::reference_wrapper<ShapeData> shapeData();

        /** @brief Underlying DART `ShapeNode` */
        dart::dynamics::ShapeNode* shapeNode() { return _node; }

        /** @brief Underlying DART `BodyNode` */
        dart::dynamics::BodyNode* bodyNode() { return _body; }

    private:
        explicit Object(SceneGraph::AbstractBasicObject3D<Float>& object, SceneGraph::AbstractBasicTranslationRotation3D<Float>& transformation, dart::dynamics::ShapeNode* node, dart::dynamics::BodyNode* body);

        /**
        @brief Convert `ShapeNode` to mesh and material data

        Returns @ref Corrade::Containers::NullOpt if the shape of given `ShapeNode` is
        not supported. The following DART shapes are supported:

        -   `BoxShape`
        -   `CapsuleShape`
        -   `CylinderShape`
        -   `EllipsoidShape`
        -   `MeshShape`
        -   `SoftMeshShape`
        -   `SphereShape`

        The following DART shapes are not yet supported:

        -   `ConeShape`
        -   `LineSegmentShape`
        -   `MultiSphereConvexHullShape`
        -   `PlaneShape` (this is an infinite plane with normal)
        */
        bool _convertShapeNode();

        SceneGraph::AbstractBasicTranslationRotation3D<Float>& _transformation;
        dart::dynamics::ShapeNode* _node;
        dart::dynamics::BodyNode* _body;
        std::unique_ptr<ShapeData> _shapeData;
        bool _used;
};

}}

#endif
