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

#include <Corrade/Utility/Directory.h>

#include <Magnum/Mesh.h>
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

Containers::Optional<ShapeData> convertShapeNode(dart::dynamics::ShapeNode& shapeNode, ShapeLoadTypes loadType, Trade::AbstractImporter* importer) {
    dart::dynamics::ShapePtr shape = shapeNode.getShape();

    bool firstTime = static_cast<bool>(loadType & ShapeLoadType::All);
    bool getMaterial = firstTime || static_cast<bool>(loadType & ShapeLoadType::Material);
    bool getPrimitive = firstTime || static_cast<bool>(loadType & ShapeLoadType::Primitive);
    bool getMesh = firstTime || static_cast<bool>(loadType & ShapeLoadType::Mesh);

    ShapeData shapeData{{}, {}, {}, {}};

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
            shapeData.materials = Containers::Array<Trade::PhongMaterialData>(Containers::NoInit, 1);
            new(&shapeData.materials[0]) Trade::PhongMaterialData{std::move(nodeMaterial)};
        }
    }

    if(getPrimitive && shape->getType() == dart::dynamics::BoxShape::getStaticType()) {
        auto boxShape = std::static_pointer_cast<dart::dynamics::BoxShape>(shape);
        Eigen::Vector3d size = boxShape->getSize();

        /* Multiplying by 0.5f because the Primitives::Cube is 2x2x2 */
        shapeData.scaling = Vector3(size(0), size(1), size(2)) * 0.5f;

        if(firstTime) {
            shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
            new(&shapeData.meshes[0]) Trade::MeshData3D{Primitives::Cube::solid()};
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::CapsuleShape::getStaticType()) {
        auto capsuleShape = std::static_pointer_cast<dart::dynamics::CapsuleShape>(shape);

        Float r = Float(capsuleShape->getRadius());

        shapeData.scaling = Vector3{r};

        if(firstTime) {
            Float h = Float(capsuleShape->getHeight());
            Float halfLength = 0.5f * h / r;

            shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
            new(&shapeData.meshes[0]) Trade::MeshData3D{Primitives::Capsule3D::solid(32, 32, 32, halfLength)};
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::CylinderShape::getStaticType()) {
        auto cylinderShape = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape);

        Float r = Float(cylinderShape->getRadius());

        shapeData.scaling = Vector3{r};

        if(firstTime) {
            Float h = Float(cylinderShape->getHeight());
            Float halfLength = 0.5f * h / r;

            shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
            new(&shapeData.meshes[0]) Trade::MeshData3D{Primitives::Cylinder::solid(32, 32, halfLength, Primitives::Cylinder::Flag::CapEnds)};
        }
    } else if(getPrimitive && shape->getType() == dart::dynamics::EllipsoidShape::getStaticType()) {
        auto ellipsoidShape = std::static_pointer_cast<dart::dynamics::EllipsoidShape>(shape);

        Eigen::Vector3d size = ellipsoidShape->getDiameters().array() * 0.5;
        shapeData.scaling = Vector3(size(0), size(1), size(2));

        if(firstTime) {
            shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
            new(&shapeData.meshes[0]) Trade::MeshData3D{Primitives::Icosphere::solid(5)};
        }
    } else if((getPrimitive || getMesh || getMaterial) && shape->getType() == dart::dynamics::MeshShape::getStaticType()) {
        if (!importer)
            return Containers::NullOpt;
        auto meshShape = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

        if(getPrimitive) {
            /* @todo: check if scaling can be per mesh */
            Eigen::Vector3d scale = meshShape->getScale();
            shapeData.scaling = Vector3(scale(0), scale(1), scale(2));
        }

        const aiScene* aiMesh = meshShape->getMesh();
        std::string meshPath = Utility::Directory::path(meshShape->getMeshPath());

        bool loaded = importer->openState(aiMesh, meshPath);
        if(!loaded || importer->mesh3DCount() < 1) {
            Debug {} << "Could not load multi mesh!";
            return Containers::NullOpt;
        }

        UnsignedInt meshesCount = 0;
        for(UnsignedInt i = 0; i < importer->object3DCount(); i++) {
            auto meshData3D = dynamic_cast<Trade::MeshObjectData3D*>(importer->object3D(i).release());
            if(meshData3D) {
                meshesCount++;
                /* delete no longer used MeshObjectData3D */
                delete meshData3D;
            }
        }

        Containers::Array<Trade::MeshData3D> meshes(Containers::NoInit, meshesCount);
        Containers::Array<Trade::PhongMaterialData> materials(Containers::NoInit, meshesCount);

        UnsignedInt j = 0;
        for(UnsignedInt i = 0; i < importer->object3DCount(); i++) {
            auto meshData3D = dynamic_cast<Trade::MeshObjectData3D*>(importer->object3D(i).release());
            if(meshData3D) {
                Containers::Optional<Trade::MeshData3D> meshData = importer->mesh3D(meshData3D->instance());
                if(!meshData)
                    return Containers::NullOpt;

                if(getMaterial) {
                    auto colorMode = meshShape->getColorMode();
                    /* only get materials from mesh if the appropriate color mode */
                    if(importer->materialCount() && getMaterial) {
                        if(colorMode == dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR) {
                            auto matPtr = importer->material(meshData3D->material());
                            new(&materials[j]) Trade::PhongMaterialData{std::move(*static_cast<Trade::PhongMaterialData*>(matPtr.get()))};
                        }
                        else if(colorMode == dart::dynamics::MeshShape::ColorMode::COLOR_INDEX) {
                            /* @todo: check if index is within bounds */
                            /* get diffuse color from Mesh color */
                            Color4 meshColor = meshData->colors(0)[meshShape->getColorIndex()];
                            new(&materials[j]) Trade::PhongMaterialData{Trade::PhongMaterialData::Flags{}, 80.f};
                            materials[j].diffuseColor() = Color3(meshColor[0], meshColor[1], meshColor[2]);
                            /* default colors for ambient (black) and specular (white) */
                            materials[j].ambientColor() = Vector3{0.f, 0.f, 0.f};
                            materials[j].specularColor() = Vector3{1.f, 1.f, 1.f};
                        }
                        else if (colorMode == dart::dynamics::MeshShape::ColorMode::SHAPE_COLOR) {
                            new(&materials[j]) Trade::PhongMaterialData{std::move(nodeMaterial)};
                        }
                    }
                }
                /* fallback material to by-pass seg-faults */
                else
                    new(&materials[j]) Trade::PhongMaterialData{Trade::PhongMaterialData::Flags{}, 80.f};

                if(getMesh) {
                    new(&meshes[j]) Trade::MeshData3D{std::move(*meshData)};
                }
                /* fallback mesh data to by-pass seg-faults */
                else
                    new(&meshes[j]) Trade::MeshData3D{MeshPrimitive::Triangles, std::vector<UnsignedInt>(), std::vector<std::vector<Vector3>>(1, std::vector<Vector3>()), std::vector<std::vector<Vector3>>(), std::vector<std::vector<Vector2>>(), std::vector<std::vector<Color4>>()};

                j++;

                /* delete no longer used MeshObjectData3D */
                delete meshData3D;
            }
        }

        Containers::Array<Containers::Optional<Trade::TextureData>> textures(importer->textureCount());
        Containers::Array<Containers::Optional<Trade::ImageData2D>> images(importer->textureCount());

        if(getMaterial) {
            for(UnsignedInt i = 0; i < importer->textureCount(); ++i) {
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

                textures[i] = std::move(textureData);
                images[i] = std::move(imageData);
            }
        }

        if(getMesh) {
            shapeData.meshes = std::move(meshes);
        }

        if(getMaterial) {
            shapeData.materials = std::move(materials);
            shapeData.images = std::move(images);
            shapeData.textures = std::move(textures);
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

        shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
        new(&shapeData.meshes[0]) Trade::MeshData3D{std::move(meshData)};
    } else if(getPrimitive && shape->getType() == dart::dynamics::SphereShape::getStaticType()) {
        auto sphereShape = std::static_pointer_cast<dart::dynamics::SphereShape>(shape);

        Float r = Float(sphereShape->getRadius());
        shapeData.scaling = Vector3{r};

        if(firstTime) {
            shapeData.meshes = Containers::Array<Trade::MeshData3D>(Containers::NoInit, 1);
            new(&shapeData.meshes[0]) Trade::MeshData3D{Primitives::Icosphere::solid(4)};
        }
    } else if(firstTime) {
        Error{} << "DartIntegration::convertShapeNode(): shape type" << shape->getType() << "is not supported";
        return Containers::NullOpt;
    }

    return std::move(shapeData);
}

}}