#ifndef UNARY_SCENE_H
#define UNARY_SCENE_H

#include <QGraphicsScene>
#include <QColor>

namespace gui
{
enum class UnaryColor
{
    Poor,
    Critical,
    Good
};

class UnaryScene : public QGraphicsScene
{
    Q_OBJECT
public:
    UnaryScene(QObject* parent = 0);
    void setTotalImages(std::size_t numImages);
    void setFrame(std::size_t id, UnaryColor color);
private:
    QColor unaryColorToQColor(UnaryColor color);
private:
    std::size_t mNumImages;
    QPen mPen;
};
} // namespace gui
#endif // UNARY_SCENE_H
