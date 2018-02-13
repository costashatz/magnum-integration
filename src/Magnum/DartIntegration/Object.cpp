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

static Eigen::Vector3d normalFromVertex(const dart::dynamics::SoftBodyNode* bn,
                                        const Eigen::Vector3i& face,
                                        std::size_t v)
{
  const Eigen::Vector3d& v0 = bn->getPointMass(face[v])->getLocalPosition();
  const Eigen::Vector3d& v1 = bn->getPointMass(face[(v+1)%3])->getLocalPosition();
  const Eigen::Vector3d& v2 = bn->getPointMass(face[(v+2)%3])->getLocalPosition();

  const Eigen::Vector3d dv1 = v1-v0;
  const Eigen::Vector3d dv2 = v2-v0;
  const Eigen::Vector3d n = dv1.cross(dv2);

  double weight = n.norm()/(dv1.norm()*dv2.norm());
  weight = std::max( -1.0, std::min( 1.0, weight) );

  return n.normalized() * asin(weight);
}

static void computeNormals(std::vector<Eigen::Vector3d>& normals,
                           const dart::dynamics::SoftBodyNode* bn)
{
  for(std::size_t i=0; i<normals.size(); ++i)
    normals[i] = Eigen::Vector3d::Zero();

  for(std::size_t i=0; i<bn->getNumFaces(); ++i)
  {
    const Eigen::Vector3i& face = bn->getFace(i);
    for(std::size_t j=0; j<3; ++j)
      normals[face[j]] += normalFromVertex(bn, face, j);
  }

  for(std::size_t i=0; i<normals.size(); ++i)
    normals[i].normalize();
}


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
        /* default scaling to (1,1,1) */
        _shapeData = std::unique_ptr<ShapeData>(new ShapeData{{}, {}, {}, {}, {}, Vector3{1.f, 1.f, 1.f}});
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

        /* Multiplying by 0.5f because the Primitives::Cube is 2x2x2 */
        _shapeData->scaling = Vector3(size(0), size(1), size(2)) * 0.5f;

        if(firstTime) {
            Trade::MeshData3D meshData{Primitives::Cube::solid()};

            /* Create the mesh */
            Mesh* mesh = new Mesh{NoCreate};
            std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
            std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);

            _shapeData->meshes[0] = mesh;
            _shapeData->vertexBuffers[0] = vertexBuffer.release();
            _shapeData->indexBuffers[0] = indexBuffer.release();
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::CapsuleShape::getStaticType()) {
        auto capsuleShape = std::static_pointer_cast<dart::dynamics::CapsuleShape>(shape);

        Float r = Float(capsuleShape->getRadius());

        _shapeData->scaling = Vector3{r};

        if(firstTime) {
            Float h = Float(capsuleShape->getHeight());
            Float halfLength = 0.5f * h / r;

            Trade::MeshData3D meshData{Primitives::Capsule3D::solid(32, 32, 32, halfLength)};

            /* Create the mesh */
            Mesh* mesh = new Mesh{NoCreate};
            std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
            std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);

            _shapeData->meshes[0] = mesh;
            _shapeData->vertexBuffers[0] = vertexBuffer.release();
            _shapeData->indexBuffers[0] = indexBuffer.release();
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::CylinderShape::getStaticType()) {
        auto cylinderShape = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape);

        Float r = Float(cylinderShape->getRadius());

        _shapeData->scaling = Vector3{r};

        if(firstTime) {
            Float h = Float(cylinderShape->getHeight());
            Float halfLength = 0.5f * h / r;

            Trade::MeshData3D meshData{Primitives::Cylinder::solid(32, 32, halfLength)};

            /* Create the mesh */
            Mesh* mesh = new Mesh{NoCreate};
            std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
            std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);

            _shapeData->meshes[0] = mesh;
            _shapeData->vertexBuffers[0] = vertexBuffer.release();
            _shapeData->indexBuffers[0] = indexBuffer.release();
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::EllipsoidShape::getStaticType()) {
        auto ellipsoidShape = std::static_pointer_cast<dart::dynamics::EllipsoidShape>(shape);

        Eigen::Vector3d size = ellipsoidShape->getDiameters().array() * 0.5;
        _shapeData->scaling = Vector3(size(0), size(1), size(2));

        if(firstTime) {
            Trade::MeshData3D meshData{Primitives::Icosphere::solid(5)};

            /* Create the mesh */
            Mesh* mesh = new Mesh{NoCreate};
            std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
            std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);

            _shapeData->meshes[0] = mesh;
            _shapeData->vertexBuffers[0] = vertexBuffer.release();
            _shapeData->indexBuffers[0] = indexBuffer.release();
        }
    } else if((getPrimitive || getMesh || getMaterial) && shape->getType() == dart::dynamics::MeshShape::getStaticType()) {
        if (!importer)
            return false;
        auto meshShape = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

        if(getPrimitive) {
            /* @todo: check if scaling can be per mesh */
            Eigen::Vector3d scale = meshShape->getScale();
            _shapeData->scaling = Vector3(scale(0), scale(1), scale(2));
        }

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

                if(getMaterial) {
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
                }

                if(getMesh) {
                    /* Create the mesh */
                    Mesh* mesh = new Mesh{NoCreate};
                    std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
                    std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(*meshData, BufferUsage::StaticDraw);

                    meshes[j] = mesh;
                    vertexBuffers[j] = vertexBuffer.release();
                    indexBuffers[j] = indexBuffer.release();
                }

                j++;
            }
        }

        Containers::Array<Texture2D*> textures(importer->textureCount());

        if(getMaterial) {
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
        }

        if(getMesh) {
            /* Delete meshes and previous buffers */
            for(UnsignedInt i = 0; i < _shapeData->meshes.size(); i++) {
                delete _shapeData->meshes[i];
                delete _shapeData->vertexBuffers[i];
                delete _shapeData->indexBuffers[i];
            }

            _shapeData->meshes = std::move(meshes);
            _shapeData->vertexBuffers = std::move(vertexBuffers);
            _shapeData->indexBuffers = std::move(indexBuffers);
        }

        if(getMaterial) {
            _shapeData->materials = std::move(materials);

            for(UnsignedInt i = 0; i < _shapeData->textures.size(); i++)
                delete _shapeData->textures[i];
            _shapeData->textures = std::move(textures);
        }

        /* Close any file if opened */
        importer->close();
    } else if(getMesh && shape->getType() == dart::dynamics::SoftMeshShape::getStaticType()) {
        /* Soft meshes contain should be drawn without face culling */
        auto meshShape = std::static_pointer_cast<dart::dynamics::SoftMeshShape>(shape);

        const dart::dynamics::SoftBodyNode* bn = meshShape->getSoftBodyNode();

        std::vector<Eigen::Vector3d> eigNormals(bn->getNumPointMasses());
        computeNormals(eigNormals, bn);

        std::vector<std::vector<Vector3>> positions, normals;
        positions.push_back(std::vector<Vector3>());
        normals.push_back(std::vector<Vector3>());

        for(UnsignedInt i = 0; i < bn->getNumFaces(); ++i) {
            const Eigen::Vector3i& F = bn->getFace(i);
            for(UnsignedInt j = 0; j < 3; ++j) {
                const Eigen::Vector3d& pos = bn->getPointMass(F[j])->getLocalPosition();
                positions[0].push_back(Vector3(pos(0), pos(1), pos(2)));
                const Eigen::Vector3d& norm = eigNormals[F[j]];
                normals[0].push_back(Vector3(norm(0), norm(1), norm(2)));
            }
        }

        Trade::MeshData3D meshData{MeshPrimitive::Triangles, std::vector<UnsignedInt>(), positions, normals, std::vector<std::vector<Vector2>>(), std::vector<std::vector<Color4>>()};

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
        _shapeData->scaling = Vector3{r};

        if(firstTime) {
            Trade::MeshData3D meshData{Primitives::Icosphere::solid(4)};

            /* Create the mesh */
            Mesh* mesh = new Mesh{NoCreate};
            std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
            std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::StaticDraw);

            _shapeData->meshes = Containers::Array<Mesh*>(1);
            _shapeData->vertexBuffers = Containers::Array<Buffer*>(1);
            _shapeData->indexBuffers = Containers::Array<Buffer*>(1);

            _shapeData->meshes[0] = mesh;
            _shapeData->vertexBuffers[0] = vertexBuffer.release();
            _shapeData->indexBuffers[0] = indexBuffer.release();
        }
    } else if(firstTime) {
        Error{} << "DartIntegration::convertShapeNode(): shape type" << shape->getType() << "is not supported";
        return false;
    }

    return true;
}

}}
