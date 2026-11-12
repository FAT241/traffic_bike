#pragma once
#include "BikeModel.h"
#include <iostream>
#include <stdexcept>
#include <utility>

// Kiểm tra display list được chuyển quyền sở hữu và thu hồi khi thay hoặc hủy lưới.
inline void runFrameResourceTests() {
    auto check = [](bool valid) {
        if (!valid)
            throw std::runtime_error("OpenGL frame resource lifetime failed");
    };
    GLuint retainedList = 0;
    {
        Frame original = ellipsoid(.2, .2, .2);
        original.draw();
        retainedList = original.displayList;
        check(glIsList(retainedList));

        Frame moved(std::move(original));
        check(original.displayList == 0 && moved.displayList == retainedList);
        Frame replacement = ellipsoid(.1, .1, .1);
        replacement.draw();
        GLuint discardedList = replacement.displayList;
        replacement = std::move(moved);
        check(moved.displayList == 0 && !glIsList(discardedList));
        check(replacement.displayList == retainedList && glIsList(retainedList));
    }
    check(!glIsList(retainedList));

    BikeModel bike;
    bike.build(BikeType::Wave, true);
    bike.draw(0, 0, false, false, false);
    std::vector<GLuint> oldLists;
    for (const auto &batch : bike.batches) {
        check(glIsList(batch.second.displayList));
        oldLists.push_back(batch.second.displayList);
    }
    bike.build(BikeType::Vision, true);
    for (GLuint list : oldLists)
        check(!glIsList(list));
    check(glGetError() == GL_NO_ERROR);
    std::cout << "RESOURCE PASS: frame move / replacement / destruction / bike rebuild\n";
}
