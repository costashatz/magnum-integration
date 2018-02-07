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

#include "ConvertShapeNode.h"
#include "Object.h"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>

namespace Magnum { namespace DartIntegration {

Object::Object(SceneGraph::AbstractBasicObject3D<Float>& object, SceneGraph::AbstractBasicTranslationRotation3D<Float>& transformation, dart::dynamics::ShapeNode* node, dart::dynamics::BodyNode* body): SceneGraph::AbstractBasicFeature3D<Float>{object}, _transformation(transformation), _node{node}, _body{body}, _used(false) {}

bool Object::_convertShapeNode() {
    /* This is not a valid object */
    if(!_node && !_body)
        return false;
    /* This object has no shape */
    if(!_node)
        return true;

    /* Convert the Shape */
    auto shapeData = convertShapeNode(*_node);
    if(!shapeData)
        return false;

    _shapeData = std::unique_ptr<ShapeData>(new ShapeData{shapeData->mesh, shapeData->vertexBuffer, shapeData->indexBuffer, std::move(shapeData->material), std::move(shapeData->textures)});

    return true;
}

Object& Object::update() {
    if(_node && (!_shapeData || _node->getShape()->getType() == dart::dynamics::SoftMeshShape::getStaticType()))
        _convertShapeNode();
    _used = true;
    /* Get transform from DART */
    const Eigen::Isometry3d* trans;
    if(!_node)
        trans = &_body->getRelativeTransform();
    else
        trans = &_node->getRelativeTransform();

    Eigen::Quaterniond quat(trans->linear());
    Eigen::Vector3d axis(quat.x(), quat.y(), quat.z());
    double angle = 2.*std::acos(quat.w());
    if(std::abs(angle)>1e-5) {
        axis = axis.array() / std::sqrt(1-quat.w()*quat.w());
        axis.normalize();
    }
    else
        axis(0) = 1.;

    Eigen::Vector3d T = trans->translation();

    /* Convert it to axis-angle representation */
    Math::Vector3<Float> t(T[0], T[1], T[2]);
    Math::Vector3<Float> u(axis(0), axis(1), axis(2));
    Rad theta(angle);

    /* Pass it to Magnum */
    _transformation.resetTransformation()
        .rotate(theta, u)
        .translate(t);

    return *this;
}

bool Object::used() {
    return _used;
}

Object& Object::clearUsed() {
    _used = false;

    return *this;
}

std::reference_wrapper<ShapeData> Object::shapeData() {
    return std::ref(*_shapeData);
}

}}
