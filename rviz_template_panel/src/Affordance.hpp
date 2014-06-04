#ifndef AFFORDANCE_HPP
#define AFFORDANCE_HPP

// qt
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QRectF>
#include <QVariant>

#include <iostream>

using namespace std;

namespace rviz_template_panel
{
    class Affordance : public QGraphicsPixmapItem
    {
    public:
        Affordance(const string& class_type, const string& image_path);
        ~Affordance() {}
        string key() const { return key_; }
    private:
        string key_;
    };
}

#endif