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

#include "World.h"

namespace Magnum { namespace DartIntegration {
    World& World::refresh() {
        if (!_dartWorld)
            return *this;
        for(size_t i = 0; i < _dartWorld->getNumSkeletons(); i++) {
            _parseSkeleton(_scene, *_dartWorld->getSkeleton(i));
        }

        return *this;
    }

    World& World::step() {
        for(auto& obj : _dartToMagnum)
            obj.second->clearUsed();

        _dartWorld->step();

        refresh();

        _toRemove = clearUnusedObjects();

        return *this;
    }

    std::vector<std::shared_ptr<Object>> World::clearUnusedObjects() {
        std::vector<dart::dynamics::Frame*> unusedFrames;

        /* Find unutilized objects */
        for(auto& object_pair : _dartToMagnum)
        {
            auto object = object_pair.second;
            if(object && !object->used())
                unusedFrames.push_back(object_pair.first);
        }

        /* Clear unused Objects */
        std::vector<std::shared_ptr<Object>> unusedObjects;
        for(dart::dynamics::Frame* frame : unusedFrames)
        {
            auto it = _dartToMagnum.find(frame);
            unusedObjects.push_back(it->second);
            /* @todo: Manage removal from scene */
            _dartToMagnum.erase(it);
        }

        return unusedObjects;
    }

    std::vector<std::shared_ptr<Object>> World::getUnusedObjects() {
        return _toRemove;
    }

    std::vector<std::shared_ptr<Object>> World::objects() {
        std::vector<std::shared_ptr<Object>> objs;
        for(auto& obj : _dartToMagnum)
            objs.emplace_back(obj.second);

        return objs;
    }

    std::vector<std::shared_ptr<Object>> World::shapeObjects() {
        std::vector<std::shared_ptr<Object>> objs;
        for(auto& obj : _dartToMagnum)
            if(obj.second->shapeNode())
                objs.emplace_back(obj.second);

        return objs;
    }

    std::vector<std::shared_ptr<Object>> World::bodyObjects() {
        std::vector<std::shared_ptr<Object>> objs;
        for(auto& obj : _dartToMagnum)
            if(obj.second->bodyNode())
                objs.emplace_back(obj.second);
        
        return objs;
    }

    std::shared_ptr<Object> World::objectFromDartFrame(dart::dynamics::Frame* frame) {
        return _dartToMagnum[frame];
    }
}}