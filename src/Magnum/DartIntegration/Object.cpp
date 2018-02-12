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

#include "Object.h"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Directory.h>

#include <Magnum/Buffer.h>
#include <Magnum/Mesh.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Texture.h>
#include <Magnum/TextureFormat.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/ObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/TextureData.h>

namespace Magnum { namespace DartIntegration {

Object::Object(SceneGraph::AbstractBasicObject3D<Float>& object, SceneGraph::AbstractBasicTranslationRotation3D<Float>& transformation, dart::dynamics::ShapeNode* node, dart::dynamics::BodyNode* body): SceneGraph::AbstractBasicFeature3D<Float>{object}, _transformation(transformation), _node{node}, _body{body}, _used(false), _updatedMesh(false) {}

Object& Object::update() {
    if(_node)
        convertShapeNode();
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

bool Object::updatedMesh() {
    return _updatedMesh;
}

std::reference_wrapper<ShapeData> Object::shapeData() {
    return std::ref(*_shapeData);
}

bool Object::convertShapeNode() {
    _updatedMesh = false;
    /* We want to load AssimpImporter only once */
    /* @todo: check if we want to put it only in the MeshShape case */
    static PluginManager::Manager<Trade::AbstractImporter> manager{MAGNUM_PLUGINS_IMPORTER_DIR};
    static std::unique_ptr<Trade::AbstractImporter> importer = manager.loadAndInstantiate("AssimpImporter");

    /* This is not a valid object */
    if(!_node && !_body)
        return false;
    /* This object has no shape */
    if(!_node)
        return true;

    unsigned int dataVariance = _node->getShape()->getDataVariance();

    if(_shapeData && dataVariance == dart::dynamics::Shape::DataVariance::STATIC)
        return true;

    // _shapeData = std::unique_ptr<ShapeData>(new ShapeData{shapeData->mesh, shapeData->vertexBuffer, shapeData->indexBuffer, std::move(shapeData->material), std::move(shapeData->textures)});
    bool firstTime = !_shapeData;
    if(firstTime){
        /* default flags for material and default shininess to 80.f */
        _shapeData = std::unique_ptr<ShapeData>(new ShapeData{{}, {}, {}, {}, {}});
    }

    dart::dynamics::ShapeNode& shapeNode = *this->shapeNode();
    dart::dynamics::ShapePtr shape = shapeNode.getShape();

    bool getMaterial = firstTime || shape->checkDataVariance(dart::dynamics::Shape::DataVariance::DYNAMIC_COLOR);
    bool getPrimitive = firstTime || shape->checkDataVariance(dart::dynamics::Shape::DataVariance::DYNAMIC_PRIMITIVE);
    bool getMesh = firstTime || shape->checkDataVariance(dart::dynamics::Shape::DataVariance::DYNAMIC_VERTICES)
                             || shape->checkDataVariance(dart::dynamics::Shape::DataVariance::DYNAMIC_ELEMENTS)
                             || shape->checkDataVariance(dart::dynamics::Shape::DataVariance::DYNAMIC);
    /* update flag for rendering */
    _updatedMesh = getMaterial || getPrimitive || getMesh;

    Trade::PhongMaterialData nodeMaterial{Trade::PhongMaterialData::Flags{}, 80.f};

    if(getMaterial) {
        /* Get material information -- we ignore the alpha value
        Note that this material is not necessarily used for the MeshShapeNodes */
        Eigen::Vector4d col = shapeNode.getVisualAspect()->getRGBA();
        /* @to-do: Create Trade Material Data that includes alpha channel */

        /* get diffuse color from Dart ShapeNode */
        nodeMaterial.diffuseColor() = Color3(col(0), col(1), col(2));
        /* default colors for ambient (black) and specular (white) */
        nodeMaterial.ambientColor() = Vector3{0.f, 0.f, 0.f};
        nodeMaterial.specularColor() = Vector3{1.f, 1.f, 1.f};

        if(shape->getType() != dart::dynamics::MeshShape::getStaticType()) {
            if(_shapeData->materials.size() != 1)
                _shapeData->materials = Containers::Array<Containers::Optional<Trade::PhongMaterialData>>(1);
            _shapeData->materials[0] = std::move(nodeMaterial);
        }
    }

    if(getPrimitive && shape->getType() == dart::dynamics::BoxShape::getStaticType()) {
        auto boxShape = std::static_pointer_cast<dart::dynamics::BoxShape>(shape);
        Eigen::Vector3d size = boxShape->getSize();

        Trade::MeshData3D meshData{Primitives::Cube::solid()};
        /* Multiplying by 0.5f because the cube is 2x2x2 */
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3(size(0), size(1), size(2))*0.5f), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if(getPrimitive && shape->getType() == dart::dynamics::CapsuleShape::getStaticType()) {
        auto capsuleShape = std::static_pointer_cast<dart::dynamics::CapsuleShape>(shape);

        Float r = Float(capsuleShape->getRadius());
        Float h = Float(capsuleShape->getHeight());
        Float halfLength = 0.5f * h / r;

        Trade::MeshData3D meshData{Primitives::Capsule3D::solid(32, 32, 32, halfLength)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if(getPrimitive && shape->getType() == dart::dynamics::CylinderShape::getStaticType()) {
        auto cylinderShape = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape);

        Float r = Float(cylinderShape->getRadius());
        Float h = Float(cylinderShape->getHeight());
        Float halfLength = 0.5f * h / r;

        Trade::MeshData3D meshData{Primitives::Cylinder::solid(32, 32, halfLength)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if(getPrimitive && shape->getType() == dart::dynamics::EllipsoidShape::getStaticType()) {
        auto ellipsoidShape = std::static_pointer_cast<dart::dynamics::EllipsoidShape>(shape);

        Eigen::Vector3d size = ellipsoidShape->getDiameters();

        Trade::MeshData3D meshData{Primitives::Icosphere::solid(5)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{Float(size(0)), Float(size(1)), Float(size(2))}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if((getPrimitive || getMesh || getMaterial) && shape->getType() == dart::dynamics::MeshShape::getStaticType()) {
        if (!importer)
            return false;
        /*  @todo check if we should not ignore the transformation in the Mesh */
        auto meshShape = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

        const aiScene* aiMesh = meshShape->getMesh();
        std::string meshPath = Utility::Directory::path(meshShape->getMeshPath());

        bool loaded = importer->openState(aiMesh, meshPath);
        if(!loaded || importer->mesh3DCount() < 1)
            return false;

        UnsignedInt meshesCount = 0;
        for(UnsignedInt i = 0; i < importer->object3DCount(); i++) {
            auto meshData3D = dynamic_cast<Trade::MeshObjectData3D*>(importer->object3D(i).release());
            if(meshData3D)
                meshesCount++;
        }

        Containers::Array<Mesh*> meshes(meshesCount);
        Containers::Array<Buffer*> vertexBuffers(meshesCount);
        Containers::Array<Buffer*> indexBuffers(meshesCount);
        Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials(meshesCount);

        UnsignedInt j = 0;
        for(UnsignedInt i = 0; i < importer->object3DCount(); i++) {
            auto meshData3D = dynamic_cast<Trade::MeshObjectData3D*>(importer->object3D(i).release());
            if(meshData3D) {
                Containers::Optional<Trade::MeshData3D> meshData = importer->mesh3D(meshData3D->instance());
                if(!meshData)
                    return false;

                Eigen::Vector3d scale = meshShape->getScale();
                /* Scale only if scaling vector is different from (1., 1., 1.) */
                if(((scale.array() - 1.).abs() > 1e-3).any())
                    MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3(scale(0), scale(1), scale(2))), meshData->positions(0));

                auto colorMode = meshShape->getColorMode();
                /* only get materials from mesh if the appropriate color mode */
                if(importer->materialCount() && getMaterial) {
                    if(colorMode == dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR) {
                        auto matPtr = importer->material(meshData3D->material());
                        materials[j] = std::move(*static_cast<Trade::PhongMaterialData*>(matPtr.get()));
                    }
                    else if(colorMode == dart::dynamics::MeshShape::ColorMode::COLOR_INDEX) {
                        /* @todo: check if index is within bounds */
                        /* get diffuse color from Mesh color */
                        Color4 meshColor = meshData->colors(0)[meshShape->getColorIndex()];
                        materials[j] = Trade::PhongMaterialData{Trade::PhongMaterialData::Flags{}, 80.f};
                        materials[j]->diffuseColor() = Color3(meshColor[0], meshColor[1], meshColor[2]);
                        /* default colors for ambient (black) and specular (white) */
                        materials[j]->ambientColor() = Vector3{0.f, 0.f, 0.f};
                        materials[j]->specularColor() = Vector3{1.f, 1.f, 1.f};
                    }
                    else if (colorMode == dart::dynamics::MeshShape::ColorMode::SHAPE_COLOR) {
                        materials[j] = std::move(nodeMaterial);
                    }
                }

                /* Create the mesh */
                Mesh* mesh = new Mesh{NoCreate};
                std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
                std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(*meshData, BufferUsage::StaticDraw);

                meshes[j] = mesh;
                vertexBuffers[j] = vertexBuffer.release();
                indexBuffers[j] = indexBuffer.release();

                j++;
            }
        }

        Containers::Array<Texture2D*> textures(importer->textureCount());

        for(UnsignedInt i = 0; i < importer->textureCount(); ++i) {
            textures[i] = nullptr;

            /* Cannot load, leave this element set to NullOpt */
            Containers::Optional<Trade::TextureData> textureData = importer->texture(i);
            if (!textureData || textureData->type() != Trade::TextureData::Type::Texture2D) {
                Warning{} << "Cannot load texture, skipping";
                continue;
            }

            /* Cannot load, leave this element set to NullOpt */
            Containers::Optional<Trade::ImageData2D> imageData = importer->image2D(textureData->image());
            if (!imageData) {
                Warning{} << "Cannot load texture image, skipping";
                continue;
            }

            auto texture = new Texture2D;
            texture->setMagnificationFilter(textureData->magnificationFilter())
                .setMinificationFilter(textureData->minificationFilter(), textureData->mipmapFilter())
                .setWrapping(textureData->wrapping().xy())
                .setStorage(1, TextureFormat::RGB8, imageData->size())
                .setSubImage(0, {}, *imageData)
                .generateMipmap();

            textures[i] = texture;
        }

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        _shapeData->meshes = std::move(meshes);
        _shapeData->vertexBuffers = std::move(vertexBuffers);
        _shapeData->indexBuffers = std::move(indexBuffers);

        if(getMaterial) {
            _shapeData->materials = std::move(materials);

            for(UnsignedInt i = 0; i < _shapeData->textures.size(); i++)
                delete _shapeData->textures[i];
            _shapeData->textures = std::move(textures);
        }

        /* Close any file if opened */
        importer->close();
    } else if((getPrimitive || getMesh) && shape->getType() == dart::dynamics::SoftMeshShape::getStaticType()) {
        /* For now soft meshes contain no normals and should be drawn without face culling */
        /* @todo: add proper normals */
        auto meshShape = std::static_pointer_cast<dart::dynamics::SoftMeshShape>(shape);

        const dart::dynamics::SoftBodyNode* bn = meshShape->getSoftBodyNode();

        std::vector<std::vector<Vector3>> positions;
        positions.push_back(std::vector<Vector3>());
        std::vector<UnsignedInt> indices;

        for(UnsignedInt i=0; i < bn->getNumPointMasses(); ++i)
        {
            const Eigen::Vector3d& pos = bn->getPointMass(i)->getLocalPosition();
            positions[0].push_back(Vector3(pos(0), pos(1), pos(2)));
        }

        for(UnsignedInt i=0; i < bn->getNumFaces(); ++i)
        {
            const Eigen::Vector3i& F = bn->getFace(i);
            for(UnsignedInt j=0; j<3; ++j)
                indices.push_back(F[j]);
        }

        Trade::MeshData3D meshData{MeshPrimitive::Triangles, indices, positions, std::vector<std::vector<Vector3>>(), std::vector<std::vector<Vector2>>(), std::vector<std::vector<Color4>>()};

        /* Create the Magnum Mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if(getPrimitive && shape->getType() == dart::dynamics::SphereShape::getStaticType()) {
        auto sphereShape = std::static_pointer_cast<dart::dynamics::SphereShape>(shape);

        Float r = Float(sphereShape->getRadius());

        Trade::MeshData3D meshData{Primitives::Icosphere::solid(4)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r, r, r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

        /* Delete meshes and previous buffers */
        for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
            delete _shapeData->meshes[i];
            delete _shapeData->vertexBuffers[i];
            delete _shapeData->indexBuffers[i];
        }

        if(_shapeData->meshes.size() != 1) {
            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);
        }

        _shapeData->meshes[0] = mesh;
        _shapeData->vertexBuffers[0] = vertexBuffer.release();
        _shapeData->indexBuffers[0] = indexBuffer.release();
    } else if(getMaterial || getPrimitive || getMesh) {
        Error{} << "DartIntegration::convertShapeNode(): shape type" << shape->getType() << "is not supported";
        return false;
    }

    return true;
}

}}
