#ifndef TOOLS_MAP_TOOLS_H
#define	TOOLS_MAP_TOOLS_H
#include "map_2d.hpp"
#include "vector_2d.hpp"
#include <vector>
#include <stack>
#include <map>
#include <limits>
namespace tools {

namespace map_tools {

template<class T>
void computeDistanceMap(const Map2D<T> *map, Map2D<float> *distance_map, const T &min_occupancy) {
    distance_map->resize(map->size_x_, map->size_y_);
    distance_map->is_init_ = true;
    distance_map->offset_x_ = map->offset_x_;
    distance_map->offset_y_ = map->offset_y_;
    distance_map->resolution_ = map->resolution_;




    Map2D <Vector2D<int> > vector_map;
    vector_map.resize(map->size_x_, map->size_y_);

    Vector2D<int> vec_N(0, 1);
    Vector2D<int> vec_NE(1, 1);
    Vector2D<int> vec_E(1, 0);
    Vector2D<int> vec_SE(1, -1);
    Vector2D<int> vec_S(0, -1);
    Vector2D<int> vec_SW(-1, -1);
    Vector2D<int> vec_W(-1, 0);
    Vector2D<int> vec_NW(-1, 1);



    // Map orientation
    // WE -> X
    // SN -> Y

    // fill vector map at the bottom and top with vector of the length 1 and direction outward
    for (int x = 0; x < map->size_x_; x++) {
        if (map->data[x][0] < min_occupancy) {
            vector_map.setCell(x, 0, Vector2D<int>());
        } else {
            vector_map.setCell(x, 0, vec_S);
        }
        if (map->data[x][map->size_y_ - 1] < min_occupancy) {
            vector_map.setCell(x, map->size_y_ - 1, Vector2D<int>());
        } else {
            vector_map.setCell(x, map->size_y_ - 1, vec_N);
        }
    }

    // fill vector map at the right and left side with vector of the length 1 and direction outward
    for (int y = 1; y < map->size_y_ - 1; y++) {

        if (map->data[0][y] < min_occupancy) {
            vector_map.setCell(0, y, Vector2D<int>());
        } else {
            vector_map.setCell(0, y, vec_W);
        }

        if (map->data[map->size_x_ - 1][y] < min_occupancy) {
            vector_map.setCell(map->size_x_ - 1, y, Vector2D<int>());
        } else {
            vector_map.setCell(map->size_x_ - 1, y, vec_E);
        }

    }

    // compute vector of each cell to the closed occupied cell
    // start at the lower left corner and run to upper right corner
    // 000000 first x then y-axis check for cell * only distances with X
    // X*0000
    // XXX000
    for (int y = 1; y < map->size_y_ - 1; y++) {
        for (int x = 1; x < map->size_x_ - 1; x++) {
            if (map->data[x][y] < min_occupancy) {
                vector_map.setCell(x, y, Vector2D<int>());
            } else {

                Vector2D<int> min(map->size_x_, map->size_y_);
                Vector2D<int> tmp;

                tmp = vector_map.getCell(x - 1, y) + vec_W;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x - 1, y - 1) + vec_SW;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x, y - 1) + vec_S;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x + 1, y - 1) + vec_SE;
                if (tmp < min) {
                    min = tmp;
                }

                vector_map.setCell(x, y, min);
            }
        }
    }

    // compute vector of each cell to the closed occupied cell
    // start at the upper right corner and run to lower left corner
    // 000XXX first x then y-axis check for cell * only distances with X
    // 0000*X
    // 000000
    for (int y = map->size_y_ - 2; y >= 1; y--) {
        for (int x = map->size_x_ - 2; x >= 1; x--) {
            if (map->data[x][y] < min_occupancy) {
                vector_map.setCell(x, y, Vector2D<int>());
            } else {

                Vector2D<int> min = vector_map.getCell(x, y);
                Vector2D<int> tmp;

                tmp = vector_map.getCell(x + 1, y) + vec_E;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x + 1, y + 1) + vec_NE;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x, y + 1) + vec_N;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map.getCell(x - 1, y + 1) + vec_NW;
                if (tmp < min) {
                    min = tmp;
                }

                vector_map.setCell(x, y, min);
            }
        }
    }

    // compute the distances by getting the norm of the vectors and multiply with the resolution
    for (int x = 0; x < map->size_x_; x++) {
        for (int y = 0; y < map->size_y_; y++) {
            distance_map->setCell(x, y, vector_map.getValue(x, y) * map->resolution_);
        }
    }
}

template<class T>
void computeVectorMap(const Map2D<T> *map, Map2D<Vector2D<int> > *vector_map, const T &min_occupancy) {
    vector_map->resize(map->size_x_, map->size_y_);
    vector_map->is_init_ = true;
    vector_map->offset_x_ = map->offset_x_;
    vector_map->offset_y_ = map->offset_y_;
    vector_map->resolution_ = map->resolution_;


    vector_map->resize(map->size_x_, map->size_y_);

    Vector2D<int> vec_N(0, 1);
    Vector2D<int> vec_NE(1, 1);
    Vector2D<int> vec_E(1, 0);
    Vector2D<int> vec_SE(1, -1);
    Vector2D<int> vec_S(0, -1);
    Vector2D<int> vec_SW(-1, -1);
    Vector2D<int> vec_W(-1, 0);
    Vector2D<int> vec_NW(-1, 1);



    // Map orientation
    // WE -> X
    // SN -> Y

    // fill vector map at the bottom and top with vector of the length 1 and direction outward
    for (int x = 0; x < map->size_x_; x++) {
        if (map->data[x][0] < min_occupancy) {
            vector_map->setCell(x, 0, Vector2D<int>());
        } else {
            vector_map->setCell(x, 0, Vector2D<int>(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()));
        }
        if (map->data[x][map->size_y_ - 1] < min_occupancy) {
            vector_map->setCell(x, map->size_y_ - 1, Vector2D<int>());
        } else {
            vector_map->setCell(x, map->size_y_ - 1, Vector2D<int>(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()));
        }
    }

    // fill vector map at the right and left side with vector of the length 1 and direction outward
    for (int y = 1; y < map->size_y_ - 1; y++) {

        if (map->data[0][y] < min_occupancy) {
            vector_map->setCell(0, y, Vector2D<int>());
        } else {
            vector_map->setCell(0, y, Vector2D<int>(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()));
        }

        if (map->data[map->size_x_ - 1][y] < min_occupancy) {
            vector_map->setCell(map->size_x_ - 1, y, Vector2D<int>());
        } else {
            vector_map->setCell(map->size_x_ - 1, y, Vector2D<int>(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()));
        }

    }

    // compute vector of each cell to the closed occupied cell
    // start at the lower left corner and run to upper right corner
    // 000000 first x then y-axis check for cell * only distances with X
    // X*0000
    // XXX000
    for (int y = 1; y < map->size_y_ - 1; y++) {
        for (int x = 1; x < map->size_x_ - 1; x++) {
            if (map->data[x][y] < min_occupancy) {
                vector_map->setCell(x, y, Vector2D<int>());
            } else {

                Vector2D<int> min(map->size_x_, map->size_y_);
                Vector2D<int> tmp;

                tmp = vector_map->getCell(x - 1, y) + vec_W;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x - 1, y - 1) + vec_SW;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x, y - 1) + vec_S;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x + 1, y - 1) + vec_SE;
                if (tmp < min) {
                    min = tmp;
                }

                vector_map->setCell(x, y, min);
            }
        }
    }

    // compute vector of each cell to the closed occupied cell
    // start at the upper right corner and run to lower left corner
    // 000XXX first x then y-axis check for cell * only distances with X
    // 0000*X
    // 000000
    for (int y = map->size_y_ - 2; y >= 1; y--) {
        for (int x = map->size_x_ - 2; x >= 1; x--) {
            if (map->data[x][y] < min_occupancy) {
                vector_map->setCell(x, y, Vector2D<int>());
            } else {

                Vector2D<int> min = vector_map->getCell(x, y);
                Vector2D<int> tmp;

                tmp = vector_map->getCell(x + 1, y) + vec_E;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x + 1, y + 1) + vec_NE;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x, y + 1) + vec_N;
                if (tmp < min) {
                    min = tmp;
                }

                tmp = vector_map->getCell(x - 1, y + 1) + vec_NW;
                if (tmp < min) {
                    min = tmp;
                }

                vector_map->setCell(x, y, min);
            }
        }
    }
}


/*
 * 2D Grid map with 0,0 at the lower left corner
 * x-axis is horizontal in pgm file
 * y-axis is vertical in pgm file
 */
template<class T>
void readFromPGM(std::istream &is, const double &resolution, bool normalize, Map2D<T> *map) {

    map->resolution_ = resolution;
    map->offset_x_ = map->offset_y_ = 0;

    std::string tag;
    is >> tag;
    if (tag != "P5") {
        std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
        exit(-1);
    }

    int new_size_x, new_size_y;

    while (is.peek() == ' ' || is.peek() == '\n')
        is.ignore();
    while (is.peek() == '#')
        is.ignore(255, '\n');
    is >> new_size_x;
    while (is.peek() == '#')
        is.ignore(255, '\n');
    is >> new_size_y;
    while (is.peek() == '#')
        is.ignore(255, '\n');
    is >> tag;
    if (tag != "255") {
        std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
        exit(-1);
    }

    map->resize(new_size_x, new_size_y);

    if (is.peek() == '\n') {
        is.get(); // remove the new line
    }

    for (int y = map->size_y_ - 1; y >= 0; y--) {
        for (int x = 0; x < map->size_x_; x++) {
            int c = is.get();

            if (normalize) {
                map->setCell(x, y, (T) c / 255.0);
            } else {
                map->setCell(x, y, (T) c);
            }

            if (!is.good()) {
                std::cerr << "Error reading pgm map.\n";
                exit(-1);
            }
        }

    }

    map->is_init_ = true;
}


} /* namespace map_tools */

} /* namespace tools */

#endif	/* MAP_TOOLS_H */

