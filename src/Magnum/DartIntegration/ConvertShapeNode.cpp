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

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
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
#include <Magnum/MeshTools/CombineIndexedArrays.h>
#include <Magnum/MeshTools/GenerateFlatNormals.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "Object.h"

namespace Magnum { namespace DartIntegration {

Containers::Optional<ShapeData> convertShapeNode(dart::dynamics::ShapeNode& shapeNode) {
    /* We only need to load AssimpImporter once */
    static PluginManager::Manager<Trade::AbstractImporter> manager{MAGNUM_PLUGINS_IMPORTER_DIR};
    static std::unique_ptr<Trade::AbstractImporter> importer = manager.loadAndInstantiate("AssimpImporter");

    dart::dynamics::ShapePtr shape = shapeNode.getShape();

    /* Get material information -- we ignore the alpha value
       Note that if this is a MeshShape we will ignore this material */
    Eigen::Vector4d col = shapeNode.getVisualAspect()->getRGBA();
    /* @to-do: Create Trade Material Data that includes alpha channel */

    /* default shininess to 80.f */
    Trade::PhongMaterialData matData{Trade::PhongMaterialData::Flags{}, 80.f};
    /* get diffuse color from Dart ShapeNode */
    matData.diffuseColor() = Color3(col(0), col(1), col(2));
    /* default colors for ambient (black) and specular (white) */
    matData.ambientColor() = Vector3{0.f, 0.f, 0.f};
    matData.specularColor() = Vector3{1.f, 1.f, 1.f};

    if(shape->getType() == dart::dynamics::BoxShape::getStaticType()) {
        auto boxShape = std::static_pointer_cast<dart::dynamics::BoxShape>(shape);
        Eigen::Vector3d size = boxShape->getSize();

        Trade::MeshData3D meshData{Primitives::Cube::solid()};
        /* Multiplying by 0.5f because the cube is 2x2x2 */
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3(size(0), size(1), size(2))*0.5f), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::DynamicDraw);

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    if(shape->getType() == dart::dynamics::CapsuleShape::getStaticType()) {
        auto capsuleShape = std::static_pointer_cast<dart::dynamics::CapsuleShape>(shape);

        Float r = Float(capsuleShape->getRadius());
        Float h = Float(capsuleShape->getHeight());
        Float halfLength = 0.5f * h / r;

        Trade::MeshData3D meshData{Primitives::Capsule3D::solid(32, 32, 32, halfLength)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::DynamicDraw);

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    if(shape->getType() == dart::dynamics::CylinderShape::getStaticType()) {
        auto cylinderShape = std::static_pointer_cast<dart::dynamics::CylinderShape>(shape);

        Float r = Float(cylinderShape->getRadius());
        Float h = Float(cylinderShape->getHeight());
        Float halfLength = 0.5f * h / r;

        Trade::MeshData3D meshData{Primitives::Cylinder::solid(32, 32, halfLength)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::DynamicDraw);

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    if(shape->getType() == dart::dynamics::EllipsoidShape::getStaticType()) {
        auto ellipsoidShape = std::static_pointer_cast<dart::dynamics::EllipsoidShape>(shape);

        Eigen::Vector3d size = ellipsoidShape->getDiameters();

        Trade::MeshData3D meshData{Primitives::Icosphere::solid(5)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{Float(size(0)), Float(size(1)), Float(size(2))}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::DynamicDraw);

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    if(shape->getType() == dart::dynamics::MeshShape::getStaticType()) {
        if (!importer)
            return Containers::NullOpt;
        /*  @todo check if we should not ignore the transformation in the Mesh */
        auto meshShape = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

        const aiScene* aiMesh = meshShape->getMesh();
        std::string meshPath = Utility::Directory::path(meshShape->getMeshPath());

        bool loaded = importer->openState(aiMesh, meshPath);
        if(!loaded || importer->mesh3DCount() < 1)
            return Containers::NullOpt;

        /* Most probably it does not make sense that one ShapeNode has multiple
           materials */
        if(importer->materialCount()) {
            auto matPtr = importer->material(0);
            matData = std::move(*static_cast<Trade::PhongMaterialData*>(matPtr.get()));
        }

        /* Most probably it does not make sense that one ShapeNode has multiple
           meshes */
        Containers::Optional<Trade::MeshData3D> meshData = importer->mesh3D(0);
        if(!meshData)
            return Containers::NullOpt;

        Eigen::Vector3d scale = meshShape->getScale();
        /* Scale only if scaling vector is different from (1., 1., 1.) */
        if(((scale.array() - 1.).abs() > 1e-3).any())
            MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3(scale(0), scale(1), scale(2))), meshData->positions(0));

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

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(*meshData, BufferUsage::DynamicDraw);

        /* Close any file if opened */
        importer->close();

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), std::move(textures)};
    }

    if(shape->getType() == dart::dynamics::SoftMeshShape::getStaticType()) {
        if (!importer)
            return Containers::NullOpt;
        auto meshShape = std::static_pointer_cast<dart::dynamics::SoftMeshShape>(shape);

        /* get aiMesh from SoftBodyNode */
        const aiMesh* assimpMesh = meshShape->getAssimpMesh();
        /* Create an aiScene for AssimpImporter to read */
        /* create the root node */
        aiNode* rootNode = new aiNode(shapeNode.getName() + "_root");
        rootNode->mNumMeshes = 1;
        rootNode->mMeshes = new unsigned int[1];
        rootNode->mMeshes[0] = 0;
        aiScene* assimpScene = new aiScene;
        assimpScene->mRootNode = rootNode;
        /* add the mesh */
        assimpScene->mNumMeshes = 1;
        assimpScene->mMeshes = new aiMesh*[1];
        /* copy the mesh into temp variable */
        aiMesh* tmpMesh = new aiMesh;
        *tmpMesh = *assimpMesh;
        tmpMesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
        assimpScene->mMeshes[0] = tmpMesh;

        bool loaded = importer->openState(assimpScene);
        if(!loaded) {
            /* delete aiScene if not loaded */
            delete assimpScene;
        }
        if(!loaded || importer->mesh3DCount() < 1)
            return Containers::NullOpt;

        /* SoftBodyNodes have only one mesh */
        Containers::Optional<Trade::MeshData3D> meshData = importer->mesh3D(0);
        if(!meshData)
            return Containers::NullOpt;

        /* Generate normals
         * there seems to be a small issue */
        std::vector<UnsignedInt> normalIndices;
        std::vector<Vector3> normals;
        std::tie(normalIndices, normals) = MeshTools::generateFlatNormals(meshData->indices(), meshData->positions(0));

        std::vector<UnsignedInt> indices = MeshTools::combineIndexedArrays(
            std::make_pair(std::cref(meshData->indices()), std::ref(meshData->positions(0))),
            std::make_pair(std::cref(normalIndices), std::ref(normals))
        );

        meshData->indices() = indices;
        meshData->normals(0) = normals;

        /* Create the Magnum Mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(*meshData, BufferUsage::DynamicDraw);

        /* Close any file if opened */
        importer->close();

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    if(shape->getType() == dart::dynamics::SphereShape::getStaticType()) {
        auto sphereShape = std::static_pointer_cast<dart::dynamics::SphereShape>(shape);

        Float r = Float(sphereShape->getRadius());

        Trade::MeshData3D meshData{Primitives::Icosphere::solid(4)};
        MeshTools::transformPointsInPlace(Matrix4::scaling(Vector3{r, r, r}), meshData.positions(0));

        /* Create the mesh */
        Mesh* mesh = new Mesh{NoCreate};
        std::unique_ptr<Buffer> vertexBuffer, indexBuffer;
        std::tie(*mesh, vertexBuffer, indexBuffer) = MeshTools::compile(meshData, BufferUsage::DynamicDraw);

        return ShapeData{mesh, vertexBuffer.release(), indexBuffer.release(), std::move(matData), {}};
    }

    Error{} << "DartIntegration::convertShapeNode(): shape type" << shape->getType() << "is not supported";
    return Containers::NullOpt;
}

Containers::Optional<ShapeData> convertShapeNode(Object& object) {
    if(!object.shapeNode())
        return Containers::NullOpt;
    return convertShapeNode(*object.shapeNode());
}

}}
