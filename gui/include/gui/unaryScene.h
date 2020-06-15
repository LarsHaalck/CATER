#ifndef UNARY_SCENE_H
#define UNARY_SCENE_H

#include <QGraphicsScene>
#include <QColor>
#include <map>

namespace gui
{
enum class UnaryQuality
{
    Good,
    Poor,
    Critical,
    Undefined,
    Excellent
};

enum class UnaryState
{
    Default,
    DefaultAlt,
    Computing,
    ComputingAlt
};


class UnaryScene : public QGraphicsScene
{
    Q_OBJECT
public:
    UnaryScene(QObject* parent = 0);
    void setTotalImages(std::size_t numImages);
    std::size_t getTotalImages() const { return mNumImages; }

    void setUnaryQuality(std::size_t id, UnaryQuality color);
    void setUnaryState(std::size_t id, UnaryState state);

    UnaryQuality getUnaryQuality(std::size_t id) const;
    QColor getUnaryColor(std::size_t id) const;
    static std::string unaryQualityToString(UnaryQuality quality);
public slots:
    void update();
private:
    QColor unaryQualityToQColor(UnaryQuality quality) const;
    QColor unaryStateToQColor(UnaryState state) const;
    inline QPen getPen(const QColor& color) const;
    inline double idToX(std::size_t idx) const;
private:
    std::size_t mNumImages;
    std::map<std::size_t, UnaryQuality> mUnaryColors;
    std::map<std::size_t, UnaryState> mUnaryStates;
    QPen mPen;
};
} // namespace gui
#endif // UNARY_SCENE_H
