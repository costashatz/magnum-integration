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

#include <assimp/defs.h> /* in assimp 3.0, version.h is missing this include for ASSIMP_API */
#include <assimp/version.h>
#include <dart/dynamics/BallJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.hpp>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/TextureData.h>

#include "Magnum/DartIntegration/ConvertShapeNode.h"
#include "Magnum/DartIntegration/Skeleton.h"
#include "Magnum/DartIntegration/Test/configure.h"

#define DART_URDF (MAGNUM_DART_URDF_FOUND > 0 && DART_MAJOR_VERSION >= 6)
#if DART_URDF
    #if DART_MINOR_VERSION < 4
        #include <dart/utils/urdf/urdf.hpp>
    #else
        #include <dart/io/urdf/urdf.hpp>
    #endif
#endif

namespace Magnum { namespace DartIntegration { namespace Test {

namespace {

using namespace Math::Literals;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

constexpr Double DefaultHeight = 1.0; // m
constexpr Double DefaultWidth = 0.2; // m
constexpr Double DefaultDepth = 0.2; // m

constexpr Double DefaultRestPosition = 0.0;

constexpr Double DefaultStiffness = 0.0;

constexpr Double DefaultDamping = 5.0;

void setGeometry(const dart::dynamics::BodyNodePtr& bn) {
    /* Create a BoxShape to be used for both visualization and collision checking */
    std::shared_ptr<dart::dynamics::BoxShape> box(new dart::dynamics::BoxShape(
        Eigen::Vector3d(DefaultWidth, DefaultDepth, DefaultHeight)));

    /* Create a shape node for visualization and collision checking */
    auto shapeNode = bn->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    /* Set the location of the shape node */
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, DefaultHeight/2.0);
    box_tf.translation() = center;
    shapeNode->setRelativeTransform(box_tf);

    /* Move the center of mass to the center of the object */
    bn->setLocalCOM(center);
}

dart::dynamics::BodyNode* makeRootBody(const dart::dynamics::SkeletonPtr& pendulum, std::string name) {
    dart::dynamics::BallJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mRestPositions = Eigen::Vector3d::Constant(DefaultRestPosition);
    properties.mSpringStiffnesses = Eigen::Vector3d::Constant(DefaultStiffness);
    properties.mDampingCoefficients = Eigen::Vector3d::Constant(DefaultDamping);

    dart::dynamics::BodyNodePtr bn = pendulum->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
        nullptr, properties, dart::dynamics::BodyNode::AspectProperties(name)).second;

    /* Make a shape for the Joint */
    auto ball = std::make_shared<dart::dynamics::EllipsoidShape>(Math::sqrt(2.0)*
        Eigen::Vector3d(DefaultWidth, DefaultWidth, DefaultWidth));
    auto shapeNode = bn->createShapeNodeWith<dart::dynamics::VisualAspect>(ball);
    auto va = shapeNode->getVisualAspect();
    CORRADE_INTERNAL_ASSERT(va);
    va->setColor(dart::Color::Blue());

    /* Set the geometry of the Body */
    setGeometry(bn);
    return bn;
}

dart::dynamics::BodyNode* addBody(const dart::dynamics::SkeletonPtr& pendulum, dart::dynamics::BodyNode* parent, std::string name) {
    /* Set up the properties for the Joint */
    dart::dynamics::RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, DefaultHeight);
    properties.mRestPositions[0] = DefaultRestPosition;
    properties.mSpringStiffnesses[0] = DefaultStiffness;
    properties.mDampingCoefficients[0] = DefaultDamping;

    /* Create a new BodyNode, attached to its parent by a RevoluteJoint */
    dart::dynamics::BodyNodePtr bn = pendulum->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
        parent, properties, dart::dynamics::BodyNode::AspectProperties(name)).second;

    /* Make a shape for the Joint */
    auto cyl = std::make_shared<dart::dynamics::CylinderShape>(DefaultWidth/2.0, DefaultDepth);

    /* Line up the cylinder with the Joint axis */
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(
        Eigen::Vector3d(Double(Radd(90.0_deg)), 0, 0));

    auto shapeNode = bn->createShapeNodeWith<dart::dynamics::VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);

    /* Set the geometry of the Body */
    setGeometry(bn);
    return bn;
}

}

struct DartSkeletonTest: TestSuite::Tester {
    explicit DartSkeletonTest();

    void test();
    void urdf();
    void texture();
};

DartSkeletonTest::DartSkeletonTest() {
    addTests({&DartSkeletonTest::test,
              #if DART_URDF
              &DartSkeletonTest::urdf,
              &DartSkeletonTest::texture
              #endif
              });
}

void DartSkeletonTest::test() {
    /* Create an empty Skeleton with the name "pendulum" */
    const std::string name = "pendulum";
    dart::dynamics::SkeletonPtr pendulum = dart::dynamics::Skeleton::create(name);

    /* Add each body to the last BodyNode in the pendulum */
    dart::dynamics::BodyNode* bn = makeRootBody(pendulum, "body1");
    bn = addBody(pendulum, bn, "body2");
    bn = addBody(pendulum, bn, "body3");
    bn = addBody(pendulum, bn, "body4");
    bn = addBody(pendulum, bn, "body5");

    /* Create a world and add the pendulum to the world */
    dart::simulation::WorldPtr world(new dart::simulation::World);
    world->addSkeleton(pendulum);

    Scene3D scene;
    Object3D* obj = new Object3D{&scene};

    auto skel = std::make_shared<Skeleton>(*obj, pendulum);

    /* Set the initial joint positions so that the pendulum
       starts to swing right away */
    pendulum->getDof(1)->setPosition(Double(Radd(120.0_deg)));
    pendulum->getDof(2)->setPosition(Double(Radd(20.0_deg)));
    pendulum->getDof(3)->setPosition(Double(Radd(-50.0_deg)));

    world->step();
    skel->updateObjects();

    auto objects = skel->objects();

    Eigen::Isometry3d trans = bn->getShapeNodesWith<dart::dynamics::VisualAspect>().back()->getTransform();

    Eigen::AngleAxisd R = Eigen::AngleAxisd(trans.linear());
    Eigen::Vector3d axis = R.axis();
    Eigen::Vector3d T = trans.translation();

    /* Convert it to axis-angle representation */
    Math::Vector3<Float> t(T[0], T[1], T[2]);
    Math::Vector3<Float> u(axis(0), axis(1), axis(2));
    Rad theta(R.angle());

    Math::Matrix4<Float> transformation;
    transformation = Math::Matrix4<Float>::translation(t) * Math::Matrix4<Float>::rotation(theta, u);

    CORRADE_VERIFY(objects.size() == 15);
    CORRADE_VERIFY(objects.back().get().object().absoluteTransformationMatrix() == transformation);
}

#if DART_URDF
void DartSkeletonTest::urdf() {
    #if DART_MINOR_VERSION < 4
    dart::utils::DartLoader loader;
    #else
    dart::io::DartLoader loader;
    #endif

    const UnsignedInt assimpVersion = aiGetVersionMajor()*100 + aiGetVersionMinor();

    const std::string filename = Utility::Directory::join(DARTINTEGRATION_TEST_DIR, "urdf/test.urdf");
    auto tmp_skel = loader.parseSkeleton(filename);
    CORRADE_VERIFY(tmp_skel);

    dart::simulation::WorldPtr world(new dart::simulation::World);
    world->addSkeleton(tmp_skel);

    Scene3D scene;
    Object3D* obj = new Object3D{&scene};

    auto skel = std::make_shared<Skeleton>(*obj, tmp_skel);

    world->step();
    skel->updateObjects();
    std::size_t pts[] = {1524, 1682, 2554, 1514, 2442, 706, 3872};

    std::size_t j = 0;
    for(Object& dartObj: skel->shapeObjects()) {
        auto shape = dartObj.shapeNode()->getShape();
        CORRADE_COMPARE(shape->getType(), "MeshShape");
        auto mydata = convertShapeNode(dartObj);
        CORRADE_VERIFY(mydata);
        CORRADE_COMPARE(mydata->mesh.positions(0).size(), pts[j]);
        {
            CORRADE_EXPECT_FAIL_IF(assimpVersion < 302,
                "Old versions of Assimp do not load the material correctly");
            CORRADE_COMPARE(mydata->material.diffuseColor(), (Vector3{0.6f, 0.6f, 0.6f}));
        }
        CORRADE_COMPARE(mydata->textures.size(), 0);
        CORRADE_COMPARE(mydata->images.size(), 0);
        ++j;
    }
}

void DartSkeletonTest::texture() {
    /** @todo Possibly works with earlier versions (definitely not 3.0) */
    const UnsignedInt assimpVersion = aiGetVersionMajor()*100 + aiGetVersionMinor();
    if(assimpVersion < 302)
        CORRADE_SKIP("Current version of Assimp would not work on this test.");

    #if DART_MINOR_VERSION < 4
    dart::utils::DartLoader loader;
    #else
    dart::io::DartLoader loader;
    #endif

    const std::string filename = Utility::Directory::join(DARTINTEGRATION_TEST_DIR, "urdf/test_texture.urdf");
    auto tmp_skel = loader.parseSkeleton(filename);
    CORRADE_VERIFY(tmp_skel);

    dart::simulation::WorldPtr world(new dart::simulation::World);
    world->addSkeleton(tmp_skel);

    Scene3D scene;
    Object3D* obj = new Object3D{&scene};

    auto skel = std::make_shared<Skeleton>(*obj, tmp_skel);

    world->step();
    skel->updateObjects();

    for(Object& dartObj: skel->shapeObjects()) {
        auto shape = dartObj.shapeNode()->getShape();
        CORRADE_COMPARE(shape->getType(), "MeshShape");
        auto mydata = convertShapeNode(dartObj);
        CORRADE_VERIFY(mydata);
        CORRADE_VERIFY(!mydata->mesh.positions(0).empty());
        CORRADE_COMPARE(mydata->textures.size(), 1);
        CORRADE_COMPARE(mydata->images.size(), 1);
        CORRADE_VERIFY(mydata->textures[0]);
        CORRADE_VERIFY(mydata->images[0]);
    }
}
#endif

}}}

CORRADE_TEST_MAIN(Magnum::DartIntegration::Test::DartSkeletonTest)
