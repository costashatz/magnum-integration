#ifndef Magnum_DartIntegration_World_h
#define Magnum_DartIntegration_World_h
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
 * @brief Class @ref Magnum::DartIntegration::World
 */

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/simulation/World.hpp>

#include <Corrade/PluginManager/Manager.h>

#include <Magnum/SceneGraph/AbstractFeature.h>
#include <Magnum/SceneGraph/AbstractObject.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "Magnum/DartIntegration/Object.h"

namespace Magnum { namespace DartIntegration {

/**
@brief DART Physics World

Parses a `dart::simulation::World` for easy usage in Magnum. It basically parses all
the `dart::dynamics::Skeleton` objects in the World and keeps track of a list of
@ref DartIntegration::Object instances. It automatically captures any change that
ocurres in the `dart::simulation::World`. After each graphics @ref DartIntegration::World::refresh
the user has access to the deleted shapes via @ref DartIntegration::World::unusedObjects and
can get the update shapes (material or mesh) via @ref DartIntegration::World::updatedShapeObjects.

@section DartIntegration-World-usage Usage

Common usage is to create a `dart::simulation::WorldPtr` and then instantiate
a `DartIntegration::World` by passing a parent @ref SceneGraph::AbstractFeature and
the `dart::simulation::WorldPtr` to its constructor:

@code{.cpp}
dart::simulation::WorldPtr dartWorld = createWorldInDart();
addDartSkeletonsToDartWorld();
SceneGraph::Scene<SceneGraph::MatrixTransformation3D> scene;
SceneGraph::Object<SceneGraph::MatrixTransformation3D> object{&scene};
// object should never be a @ref SceneGraph::Scene
auto world = std::make_shared<DartIntegration::World>(object, dartWorld);
@endcode

Only the `dart::simulation::World` can affect the transformation of the Magnum
@ref DartIntegration::World and not the other way around. To update the world
and get the updated shapes you can do the following:

@code{.cpp}
std::shared_ptr<DartIntegration::World> world = createDartIntegrationWorld();
// Simulate with time step of 0.001 seconds
world->world()->setTimeStep(0.001);

for(UnsignedInt i = 0; i < simulationSteps; i++) {
    world->step();
    // update graphics at ~60Hz
    // 15*0.001 ~= 60Hz
    if(i%15 == 0) {
        world->refresh();

        // get unused/deleted shapes
        std::vector<std::shared_ptr<Object>> unusedObjects = world->unusedObjects();

        deleteObjectsFromScene(unusedObjects);

        // get updated shapes;
        // ones that either the material or the mesh pointers have changed
        std::vector<std::shared_ptr<Object>> updatedObjects = world->updatedShapeObjects();

        updatePointersAndMaterials(updatedObjects);

        // clear list of updated objects
        world->clearUpdatedShapeObjects();
    }
}
@endcode

@experimental
*/
class MAGNUM_DARTINTEGRATION_EXPORT World {
    public:
         /**
         * @brief Constructor
         * @param object    Parent object
         * @param skeleton  DART World shared pointer to parse
         */
        template<class T> explicit World(T& object, std::shared_ptr<dart::simulation::World> world): _object(object), _manager{MAGNUM_PLUGINS_IMPORTER_DIR}, _dartWorld(world) {
            objectCreator = [](SceneGraph::AbstractBasicObject3D<Float>& parent) -> SceneGraph::AbstractBasicObject3D<Float>* {
                return new T{static_cast<T*>(&parent)};
            };
            dartObjectCreator = [](SceneGraph::AbstractBasicObject3D<Float>& parent, dart::dynamics::BodyNode* body) -> std::shared_ptr<Object> {
                return std::make_shared<Object>(static_cast<T&>(parent), body);
            };
            dartShapeObjectCreator = [](SceneGraph::AbstractBasicObject3D<Float>& parent, dart::dynamics::ShapeNode* node) -> std::shared_ptr<Object> {
                return std::make_shared<Object>(static_cast<T&>(parent), node);
            };

            refresh();
        }

        /** @brief Refresh/regenerate meshes for all bodies in DART world */
        World& refresh();

        /** @brief Do a DART world step */
        World& step();

        /** @brief Cleared all objects that were not updated during the last refresh call */
        World& clearUnusedObjects();

        /** @brief Get unused objects; all objects that were not updated during the last refresh call
         * Note: this list will be cleared once a new refresh call is made
         */
        std::vector<std::shared_ptr<Object>> unusedObjects();

        /** @brief Get all @ref Objects */
        std::vector<std::shared_ptr<Object>> objects();

        /** @brief Get all @ref Objects that have shapes */
        std::vector<std::shared_ptr<Object>> shapeObjects();

        /** @brief Get all @ref Objects that have updated meshes */
        std::vector<std::shared_ptr<Object>> updatedShapeObjects();

        /** @brief Clear list of updated shape @ref Objects */
        World& clearUpdatedShapeObjects();

        /** @brief Get all @ref Objects that do not have shapes */
        std::vector<std::shared_ptr<Object>> bodyObjects();

        /** @brief Helper function to get @ref Object from a DART Frame/BodyNode/ShapeNode */
        std::shared_ptr<Object> objectFromDartFrame(dart::dynamics::Frame* frame);

        /** @brief Get DART world object
         * for making DART specific changes/updates
         */
        std::shared_ptr<dart::simulation::World> world();

    private:
        /** @brief Function to create new @ref SceneGraph::AbstractBasicObject3D of correct type */
        SceneGraph::AbstractBasicObject3D<Float>*(*objectCreator)(SceneGraph::AbstractBasicObject3D<Float>& parent);

        /** @brief Function to create new @ref Object without shape with correct parent type */
        std::shared_ptr<Object>(*dartObjectCreator)(SceneGraph::AbstractBasicObject3D<Float>& parent, dart::dynamics::BodyNode* body);

        /** @brief Function to create new @ref Object with shape with correct parent type */
        std::shared_ptr<Object>(*dartShapeObjectCreator)(SceneGraph::AbstractBasicObject3D<Float>& parent, dart::dynamics::ShapeNode* node);

        /** @brief Parse DART Skeleton and create/update shapes */
        template <class T> void parseSkeleton(T& parent, dart::dynamics::Skeleton& skel);

        /** @brief Recursively parse DART BodyNode and all of its children */
        template <class T> void parseBodyNodeRecursive(T& parent, dart::dynamics::BodyNode& bn);

        SceneGraph::AbstractBasicObject3D<Float>& _object;
        PluginManager::Manager<Trade::AbstractImporter> _manager;
        std::unique_ptr<Trade::AbstractImporter> _importer;
        std::shared_ptr<dart::simulation::World> _dartWorld;
        std::unordered_map<dart::dynamics::Frame*, std::shared_ptr<Object>> _dartToMagnum;
        std::vector<std::shared_ptr<Object>> _toRemove;
        std::unordered_set<std::shared_ptr<Object>> _updatedShapeObjects;
};

template <class T> void World::parseSkeleton(T& parent, dart::dynamics::Skeleton& skel){
    for(size_t i = 0; i < skel.getNumTrees(); i++) {
        parseBodyNodeRecursive(parent, *skel.getRootBodyNode(i));
    }
}

template <class T> void World::parseBodyNodeRecursive(T& parent, dart::dynamics::BodyNode& bn) {
    /** parse the BodyNode
     * we care only about visuals
     */
    auto& visualShapes = bn.getShapeNodesWith<dart::dynamics::VisualAspect>();

    /* create an object of the BodyNode to keep track of transformations */
    SceneGraph::AbstractBasicObject3D<Float>* object = nullptr;
    auto it = _dartToMagnum.insert(std::make_pair(static_cast<dart::dynamics::Frame*>(&bn), nullptr));
    if (it.second) {
        object = objectCreator(parent);
        it.first->second = dartObjectCreator(*object, &bn);
    }
    else
        object = static_cast<SceneGraph::AbstractBasicObject3D<Float>*>(&it.first->second->object());
    it.first->second->update();
    for (auto& shape : visualShapes) {
        auto it = _dartToMagnum.insert(std::make_pair(static_cast<dart::dynamics::Frame*>(shape), nullptr));
        if (it.second) {
            /* create objects for the ShapeNodes to keep track of inner transformations */
            auto shapeObj = objectCreator(*object);
            it.first->second = dartShapeObjectCreator(*shapeObj, shape);
        }
        it.first->second->update(_importer.get());
        if(it.first->second->hasUpdatedMesh())
            _updatedShapeObjects.insert(it.first->second);
    }

    /* parse the children recursively */
    std::size_t numChilds = bn.getNumChildBodyNodes();
    for (std::size_t i = 0; i < numChilds; i++) {
        /* pass as parent the newly created object */
        parseBodyNodeRecursive(*object, *bn.getChildBodyNode(i));
    }
}

}}

#endif
