#ifndef TOOLS_MAP_2D_HPP_
#define TOOLS_MAP_2D_HPP_

#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>

namespace tools {

/*
 * 2D Grid map with 0,0 at the lower left corner
 * x-axis is horizontal in pgm file
 * y-axis is vertical in pgm file
 */
template<class T>
class Map2D {
public:
    T **data;
    int size_x_; // weith
    int size_y_; // hight
    double offset_x_, offset_y_;
    double resolution_;
    bool is_init_;

public:

    Map2D() {
        size_x_ = -1;
        size_y_ = -1;
        offset_x_ = 0;
        offset_y_ = 0;
        data = NULL;
        is_init_ = false;
        resolution_ = 0.0;
    }

    Map2D(const Map2D<T> & other) {
        data = NULL;
        *this = other;
    }

    virtual ~Map2D() {
        if (data != NULL) {
            for (int x = 0; x < (int) size_x_; x++) {
                delete[] data[x];
            }
            delete[] data;
        }
    }

    T ** getRawData() {
        return data;
    }

    int getMapSizeX() {
        return size_x_;
    }

    int getMapSizeY() {
        return size_y_;
    }

    double getResolution() {
        return resolution_;
    }

    void setResolution(double resolution) {
        resolution_ = resolution;
    }

    double getOffsetX() {
        return offset_x_;
    }

    double getOffsetY() {
        return offset_y_;
    }

    void setOffsetXY(double x, double y) {
        offset_x_ = x;
        offset_y_ = y;
    }

    inline T getCell(int x, int y) const {
        return data[x][y];
    }

    inline bool getCell(int x, int y, T &value) const {
        if (x < 0 || y < 0 || x >= (int) size_x_ || y >= (int) size_y_) {
            return false;
        }

        value = data[x][y];

        return true;
    }

    inline T& getCellRef(int x, int y) const {
        return data[x][y];
    }

    inline double getValue(int x, int y) {
        return (double) data[x][y];
    }

    inline void setCell(int x, int y, const T & val) {
        data[x][y] = val;
    }

    inline void setCell(int x, int y, T & val) {
        data[x][y] = val;
    }

    void resize(int size_x, int size_y) {
        if (data != NULL && (size_x != size_x_ || size_y != size_y_)) {
            for (int x = 0; x < size_x_; x++) {
                delete[] data[x];
            }
            delete[] data;
            data = NULL;
        }

        if (!data) {
            data = new T*[size_x];
            size_x_ = size_x;
            size_y_ = size_y;
            for (int x = 0; x < size_x_; x++) {
                data[x] = new T[size_y];
            }
        }
        is_init_ = false;
    }

    bool fill(const std::vector<signed char>& data) {
        if (data.size() == 0) {
            std::cerr << "tools::Map2D -> fill -> Map size == 0!" << std::endl;
            return false;
        }

        for (unsigned int i = 0; i < data.size(); i++) {

            int y = i / size_x_;
            int x = i % size_x_;

            char cell = data[i];

            // check this if it is working with ros msgs
            double d;
            if (cell >= 0) {
                d = 1 - (cell / 100.);
            } else {
                d = 1;
                // d = 0; // Hack!!!
            }
            this->data[x][y] = d;

            //        std::cout << d  << ", ";

        }

        is_init_ = true;
        return true;
    }

    bool writeToPGM(const char* filename, bool normalize = true) {
        double max = 1;
        if (normalize) {
            for (int x = 0; x < size_x_; x++) {
                for (int y = 0; y < size_y_; y++) {
                    double val = data[x][y];
                    if (val > max)
                        max = val;
                }
            }
        }
        FILE* F = fopen(filename, "w");
        if (!F) {
            printf("could not open file \"%s\" for writing!\n", filename);
            return false;
        }

        fprintf(F, "P5\n");
        fprintf(F, "%d %d \n255\n", size_x_, size_y_);
        //    for(int y = 0; y<sizeY; y++){

        //std::cout << "save map" << std::endl;
        for (int y = size_y_ - 1; y >= 0; y--) {
            for (int x = 0; x < size_x_; x++) {
                float f = 255 * (double) data[x][y] / max;
                if (f > 255)
                    f = 255;
                char c;
                if (f >= 0)
                    c = (unsigned char) f;
                else
                    c = 127 + f;
                fputc(c, F);
                //	std::cout << data[x][y] <<", ";
            }
            //std::cout <<  std::endl;
        }

        fclose(F);
        return true;
    }

    bool writeToPPM(const char* filename, bool normalize = true) {
        double max = 1;
        if (normalize) {
            for (int x = 0; x < size_x_; x++) {
                for (int y = 0; y < size_y_; y++) {
                    double val = data[x][y];
                    if (val > max)
                        max = val;
                }
            }
        }
        FILE* F = fopen(filename, "w");
        if (!F) {
            printf("could not open file \"%s\" for writing!\n", filename);
            return false;
        }

        fprintf(F, "P6\n");
        fprintf(F, "%d %d \n255\n", size_x_, size_y_);
        //    for(int y = 0; y<sizeY; y++){

        for (int y = size_y_ - 1; y >= 0; y--) {
            for (int x = 0; x < size_x_; x++) {

                float f = 255 * (double) data[x][y] / max;
                if (f > 255)
                    f = 255;
                char c;
                if (f >= 0) {
                    c = (unsigned char) f;
                    fputc(c, F);
                    fputc(c, F);
                    fputc(c, F);


                } else {
                    f = (double) data[x][y];
                    if (-10 >= f && f > -20) {
                        f = (fabs(f) - 10) * 25;
                        if (f > 255) {
                            f = 255;
                        }
                        // red
                        fputc((unsigned char) f, F);
                        fputc((unsigned char) 150, F);
                        fputc((unsigned char) 150, F);
                    } else if (-20 >= f && f > -30) {
                        // green
                        f = (fabs(f) - 20) * 25;
                        if (f > 255) {
                            f = 255;
                        }
                        fputc((unsigned char) 255 - f, F);
                        fputc((unsigned char) f, F);
                        fputc(0, F);
                    } else {

                        fputc(0, F);
                        fputc(0, F);
                        fputc(0, F);
                    }

                }

            }
        }

        fclose(F);
        return true;
    }

    bool isInit() {
        return is_init_;
    }

    bool setIsInit(bool init) {
        is_init_ = init;
        return is_init_;
    }

    bool world2map(double x, double y, int& map_x, int& map_y) {
        map_x = (int) ((x - offset_x_) / resolution_);
        map_y = (int) ((y - offset_y_) / resolution_);

        if (map_x < 0 || map_y < 0 || map_x >= (int) size_x_ || map_y >= (int) size_y_) {
            return false;
        }
        return true;
    }

    void map2world(int map_x, int map_y, double& x, double& y) {

        x = (map_x + 0.5) * resolution_ + offset_x_; // add have a cell to return the cell middle
        y = (map_y + 0.5) * resolution_ + offset_y_; // add have a cell to return the cell middle
    }

    Map2D<T>& operator =(const Map2D<T> & arg) {

        resize(arg.size_x_, arg.size_y_);

        is_init_ = arg.is_init_;
        offset_x_ = arg.offset_x_;
        offset_y_ = arg.offset_y_;
        resolution_ = arg.resolution_;

        for (int x = 0; x < size_x_; x++) {
            for (int y = 0; y < size_y_; y++) {

                setCell(x, y, arg.getCell(x, y));
            }
        }

        return *this;
    }

};

}

/* namespace tools */
#endif /* TOOLS_MAP_2D_HPP_ */
