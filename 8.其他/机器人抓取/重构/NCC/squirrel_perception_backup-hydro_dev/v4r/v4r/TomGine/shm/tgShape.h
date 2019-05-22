#ifndef TG_SHAPE_H
#define TG_SHAPE_H

#include <ak/shm/shmvector.hpp>
#include <ak/shm/shmvar.hpp>

namespace TomGine {

class ShapeAppearance {
public:
    ShapeAppearance() {};
    ShapeAppearance(const ShapeAppearance &_appearance) {
        _appearance.copyTo(*this);
    };

    ShapeAppearance(int _id, double _r, double _g, double _b, double _a = 1.) {
        id(_id);
        color(_r, _g, _b, _a);
    };
    const void copyTo(ShapeAppearance &_appearance) const {
        memcpy(&_appearance, this, sizeof(ShapeAppearance));
    }
    void color(double _r, double _g, double _b, double _a = 1.) {
        color_[0] = _r, color_[1] = _g, color_[2] = _b, color_[3] = _a;
    }
    void id(int _id) {
        id_ = _id;
    }
    const int &id() const {
        return id_;
    }
    const double &r() const {
        return color_[0];
    }
    const double &g() const {
        return color_[1];
    }
    const double &b() const {
        return color_[2];
    }
    const double &a() const {
        return color_[3];
    }
    void reset() {
        memset(this, 0, sizeof(ShapeAppearance));
    }
protected:
    int id_;
    double color_[4];
};


class ShapeEntry {
public:
    static const int BOX = 0;
    static const int SPHERE = 1;
    static const int CYLINDER = 2;
    ShapeEntry() {};
    ShapeEntry(const ShapeEntry &_shape) {
        _shape.copyTo(*this);
    };
    ShapeEntry(int _type, int _appearance) {
        type(_type);
        appearance(_appearance);
    };
    const void copyTo(ShapeEntry &_shape) const {
        memcpy(&_shape, this, sizeof(ShapeEntry));
    }
    void position(double _x, double _y, double _z) {
        position_[0] = _x, position_[1] = _y, position_[2] = _z;
    }
    void rotation(double _wx, double _wy, double _wz) {
        rotation_[0] = _wx, rotation_[1] = _wy, rotation_[2] = _wz;
    }
    void reset() {
        memset(this, 0, sizeof(ShapeEntry));
    }
    void type(int _type) {
        type_ = _type;
    }
    int type() const {
        return type_;
    }
    void appearance(int _appearance) {
        appearance_ = _appearance;
    }
    int appearance() {
        return appearance_;
    }
    const double &x() const{
      return position_[0];
    }
    const double &y() const{
      return position_[1];
    }
    const double &z() const{
      return position_[2];
    }
    const double &wx() const{
      return rotation_[0];
    }
    const double &wy() const{
      return rotation_[1];
    }
    const double &wz() const{
      return rotation_[2];
    }
protected:
    int type_;
    int appearance_;
    double position_[3];
    double rotation_[3];
    double data_[8];
    int properties_[8];
    bool switches_[8];
};

class ShapeBox : public ShapeEntry {
public:
    ShapeBox() {};
    ShapeBox(const ShapeBox &_shape) {
        _shape.copyTo(*this);
    };
    ShapeBox(const ShapeAppearance &_appearance, double _dx, double _dy, double _dz) {
        type(BOX);
        appearance(_appearance.id());
        dimension(_dx, _dy, _dz);
    };
    void dimension(double _dx, double _dy, double _dz) {
        data_[0] = _dx, data_[1] = _dy, data_[2] = _dz;
    }
    const double &dx() const {
      return data_[0];
    }
    const double &dy() const {
      return data_[1];
    }
    const double &dz() const {
      return data_[2];
    }
};
class ShapeSphere : public ShapeEntry {
public:
    ShapeSphere() {};
    ShapeSphere(const ShapeSphere &_shape) {
        _shape.copyTo(*this);
    };
    ShapeSphere(const ShapeAppearance &_appearance, double _radius, int _subdevisions = 3, int _mode = 2) {
        type(SPHERE);
        appearance(_appearance.id());
        radius(_radius);
	subdevisions(_subdevisions);
	mode(_mode);
    };
    const double radius() const {
      return data_[0];
    }
    void radius(double _radius) {
      data_[0] = _radius;
    }
    void subdevisions(int _subdevisions){
      properties_[0] = _subdevisions;
    }
    const int subdevisions() const {
      return properties_[0];
    }
    void mode(int _mode){
      properties_[1] = _mode;
    }
    const int mode() const {
      return properties_[1];
    }
};
class ShapeCylinder : public ShapeEntry {
public:
    ShapeCylinder() {};
    ShapeCylinder(const ShapeCylinder &_shape) {
        _shape.copyTo(*this);
    };
    ShapeCylinder(const ShapeAppearance &_appearance, double _radius, double _height, int _slices = 64, int _stacks = 2, bool _closed = true) {
        type(CYLINDER);
        appearance(_appearance.id());
        radius(_radius);
        height(_height);
	slices(_slices);
	stacks(_stacks);
	closed(_closed);
    };
    const double radius() const {
      return data_[0];
    }
    void radius(double _radius) {
      data_[0] = _radius;
    }
    const double height() const {
      return data_[1];
    }
    void height(double _height) {
      data_[1] = _height;
    }
    const int &slices() const {
      return properties_[0];
    }
    void slices(double _slices) {
      properties_[0] = _slices;
    }
    const int &stacks() const {
      return properties_[1];
    }
    void stacks(double _stacks) {
      properties_[1] = _stacks;
    }
    const bool &closed() const {
      return switches_[0];
    }
    void closed(bool _closed) {
      switches_[0] = _closed;
    }
};

class ShapeLine {
public:
    ShapeLine() {};
    ShapeLine(const ShapeLine &_line) {
        _line.copyTo(*this);
    };
    ShapeLine(const ShapeAppearance &_appearance, double _x1, double _y1, double _z1, double _x2, double _y2, double _z2) {
       appearance(_appearance.id());
       start(_x1, _y1, _z1);
       end(_x2, _y2, _z2);
    };
    const void copyTo(ShapeLine &_line) const {
        memcpy(&_line, this, sizeof(ShapeLine));
    }
    void start(double _x, double _y, double _z) {
        A_[0] = _x, A_[1] = _y, A_[2] = _z;
    }
    void end(double _x, double _y, double _z) {
        B_[0] = _x, B_[1] = _y, B_[2] = _z;
    }
    void reset() {
        memset(this, 0, sizeof(ShapeLine));
    }
    void appearance(int _appearance) {
        appearance_ = _appearance;
    }
    int appearance() {
        return appearance_;
    }
    const double &x1() const{
      return A_[0];
    }
    const double &y1() const{
      return A_[1];
    }
    const double &z1() const{
      return A_[2];
    }
    const double &x2() const{
      return B_[0];
    }
    const double &y2() const{
      return B_[1];
    }
    const double &z2() const{
      return B_[2];
    }
protected:
    int appearance_;
    double A_[3];
    double B_[3];
};

typedef ak::ShmVector<ShapeLine, 0> ShmTGLines;
typedef ak::ShmVector<ShapeEntry, 0> ShmTGShapes;
typedef ak::ShmVector<ShapeAppearance, 0> ShmTGAppearance;
}


#endif //TG_SHAPE_H
