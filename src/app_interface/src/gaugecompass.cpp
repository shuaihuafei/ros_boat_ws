#pragma execution_character_set("utf-8")

#include "gaugecompass.h"
#include "qpainter.h"
#include "qmath.h"
#include "qtimer.h"
#include "qdebug.h"

GaugeCompass::GaugeCompass(QWidget *parent) : QWidget(parent)
{
    value = 0;
    animation = true;
    animationStep = 1;
    precision = 0;

    crownColorStart = QColor(125, 32, 36);
    crownColorEnd = QColor(227, 77, 77);

    bgColorStart = QColor(81, 97, 113);
    bgColorEnd = QColor(28, 34, 40);

    darkColor = QColor(51, 60, 75);
    lightColor = QColor(109, 134, 162);

    foreground = QColor(255, 255, 255);
    textColor = QColor(20, 20, 20);

    northPointerColor = QColor(255, 107, 107);
    southPointerColor = QColor(100, 184, 255);

    centerColorStart = QColor(230, 230, 230);
    centerColorEnd = QColor(210, 210, 210);

    reverse = false;
    currentValue = 0;
    pre_currentValue = 0;
    timer = new QTimer(this);
    timer->setInterval(2);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateValue()));

    QFont font;
    font.setFamily("Arial");
    font.setPixelSize(11);
    setFont(font);
}

GaugeCompass::~GaugeCompass()
{
    if (timer->isActive()) {
        timer->stop();
    }
}

void GaugeCompass::paintEvent(QPaintEvent *)
{
    int width = this->width();
    int height = this->height();
    int side = qMin(width, height);

    //绘制准备工作,启用反锯齿,平移坐标轴中心,等比例缩放
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    painter.translate(width / 2, height / 2);
    painter.scale(side / 200.0, side / 200.0);

    //绘制外边框圆
    drawCrownCircle(&painter);
    //绘制背景圆
    drawBgCircle(&painter);
    //绘制刻度
    drawScale(&painter);
    //绘制东南西北标识
    drawScaleNum(&painter);
    //绘制覆盖圆外圆
    drawCoverOuterCircle(&painter);
    //绘制覆盖圆内圆
    drawCoverInnerCircle(&painter);
    //绘制覆盖圆中心圆
    drawCoverCenterCircle(&painter);
    //绘制南北指针
    drawPointer(&painter);
    //绘制中心圆
    drawCenterCircle(&painter);
    //绘制当前值
    drawValue(&painter);
}

void GaugeCompass::drawCrownCircle(QPainter *painter)
{
    int radius = 99;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, crownColorStart);
    lineGradient.setColorAt(1, crownColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawBgCircle(QPainter *painter)
{
    int radius = 90;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, bgColorStart);
    lineGradient.setColorAt(1, bgColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawScale(QPainter *painter)
{
    int radius = 85;
    painter->save();

    //总共8格,4格为NESW字母,4格为线条
    int steps = 8;
    double angleStep = 360.0 / steps;

    QPen pen;
    pen.setColor(foreground);
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidth(4);
    painter->setPen(pen);

    //%2整数部分绘制NESW字母,其余绘制线条刻度
    for (int i = 0; i <= steps; ++i) {
        if (i % 2 != 0) {
            painter->drawLine(0, radius - 10, 0, radius);
        }

        painter->rotate(angleStep);
    }

    painter->restore();
}

void GaugeCompass::drawScaleNum(QPainter *painter)
{
    int radius = 88;
    painter->save();
    painter->setPen(foreground);

    QFont font;
    font.setPixelSize(15);
    font.setBold(true);
    painter->setFont(font);

    QRect textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignTop | Qt::AlignHCenter, "N");
    painter->drawText(textRect, Qt::AlignBottom | Qt::AlignHCenter, "S");

    radius -= 2;
    textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, "W");

    radius -= 2;
    textRect = QRect(-radius, -radius, radius * 2, radius * 2);
    painter->drawText(textRect, Qt::AlignRight | Qt::AlignVCenter, "E");

    painter->restore();
}

void GaugeCompass::drawCoverOuterCircle(QPainter *painter)
{
    int radius = 68;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, lightColor);
    lineGradient.setColorAt(1, darkColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawCoverInnerCircle(QPainter *painter)
{
    int radius = 60;
    painter->save();
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, darkColor);
    lineGradient.setColorAt(1, lightColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawCoverCenterCircle(QPainter *painter)
{
    int radius = 15;
    painter->save();
    painter->setPen(Qt::NoPen);
    painter->setOpacity(0.8);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, lightColor);
    lineGradient.setColorAt(1, darkColor);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawPointer(QPainter *painter)
{
    int radius = 75;

    QPolygon pts;

    painter->save();
    painter->setOpacity(0.7);
    painter->setPen(Qt::NoPen);
    painter->setBrush(northPointerColor);
    pts.setPoints(3, -10, 0, 10, 0, 0, radius);
    painter->rotate(currentValue + 180);
    painter->drawConvexPolygon(pts);
    painter->restore();

    painter->save();
    painter->setOpacity(0.7);
    painter->setPen(Qt::NoPen);
    painter->setBrush(southPointerColor);
    pts.setPoints(3, -10, 0, 10, 0, 0, radius);
    painter->rotate(currentValue);
    painter->drawConvexPolygon(pts);
    painter->restore();
}

void GaugeCompass::drawCenterCircle(QPainter *painter)
{
    int radius = 12;
    painter->save();
    painter->setOpacity(1.0);
    painter->setPen(Qt::NoPen);
    QLinearGradient lineGradient(0, -radius, 0, radius);
    lineGradient.setColorAt(0, centerColorStart);
    lineGradient.setColorAt(1, centerColorEnd);
    painter->setBrush(lineGradient);
    painter->drawEllipse(-radius, -radius, radius * 2, radius * 2);
    painter->restore();
}

void GaugeCompass::drawValue(QPainter *painter)
{
    int radius = 100;
    painter->save();
    painter->setPen(textColor);

    QFont font;
    font.setPixelSize(11);
    font.setBold(true);
    painter->setFont(font);

    QRectF textRect(-radius, -radius, radius * 2, radius * 2);
    QString text = QString("%1").arg((double)value, 0, 'f', precision);
    painter->drawText(textRect, Qt::AlignCenter, text);
    painter->restore();
}

void GaugeCompass::updateValue()
{
    //顺时针旋转
    if (!reverse) {
        //当当前值在目标步长范围内时，说明到达目标值
        if (currentValue <= (value + animationStep) && currentValue > (value - animationStep)) {
            currentValue = value;
            pre_currentValue = currentValue;
            timer->stop();
        } else {
            //通过设定一个预当前值来先判断一下是否处于0和359.9的临界点
            pre_currentValue += animationStep;
            if (pre_currentValue >= 360)
            {
                currentValue = pre_currentValue - 360;
                pre_currentValue = currentValue;
            } else {
                currentValue = pre_currentValue;
            }
        }
    } else { //逆时针旋转
        //当当前值在目标步长范围内时，说明到达目标值
        if (currentValue < (value + animationStep) && currentValue >= (value - animationStep)) {
            currentValue = value;
            pre_currentValue = currentValue;
            timer->stop();
        } else {
            //通过设定一个预当前值来先判断一下是否处于0和359.9的临界点
            pre_currentValue -= animationStep;
            if (pre_currentValue < 0)
            {
                currentValue = pre_currentValue + 360;
                pre_currentValue = currentValue;
            } else {
                currentValue = pre_currentValue;
            }
        }
    }

    this->update();
}

QSize GaugeCompass::sizeHint() const
{
    return QSize(200, 200);
}

QSize GaugeCompass::minimumSizeHint() const
{
    return QSize(50, 50);
}

double GaugeCompass::getValue() const
{
    return this->value;
}

void GaugeCompass::setValue(double value)
{
    //值和当前值一致则无需处理
    if (value == this->value) {
        return;
    }

    //值小于最小值则取最小值,大于最大值则取最大值
    if (value < 0) {
        value = 0;
    } else if (value >= 360) {
        value = 0;
    }

    //对当前值是否大于180，分情况讨论
    if (currentValue < 180) {
        if ((value < currentValue) || (value > (currentValue + 180))) {
            reverse = true;
        } else {
            reverse = false;
        }
    } else {
        if ((value > currentValue) || (value < (currentValue - 180))) {
            reverse = false;
        } else {
            reverse = true;
        }
    }

    this->value = value;
    Q_EMIT valueChanged(value);

    if (!animation) {
        currentValue = this->value;
        this->update();
    } else {
        timer->start();
    }
}

void GaugeCompass::setValue(int value)
{
    setValue((double)value);
}

int GaugeCompass::getPrecision() const
{
    return this->precision;
}

void GaugeCompass::setPrecision(int precision)
{
    //最大精确度为 3
    if (precision <= 3 && this->precision != precision) {
        this->precision = precision;
        this->update();
    }
}

bool GaugeCompass::getAnimation() const
{
    return this->animation;
}

void GaugeCompass::setAnimation(bool animation)
{
    if (this->animation != animation) {
        this->animation = animation;
        this->update();
    }
}

double GaugeCompass::getAnimationStep() const
{
    return this->animationStep;
}

void GaugeCompass::setAnimationStep(double animationStep)
{
    if (this->animationStep != animationStep) {
        this->animationStep = animationStep;
        this->update();
    }
}

QColor GaugeCompass::getCrownColorStart() const
{
    return this->crownColorStart;
}

void GaugeCompass::setCrownColorStart(const QColor &crownColorStart)
{
    if (this->crownColorStart != crownColorStart) {
        this->crownColorStart = crownColorStart;
        this->update();
    }
}

QColor GaugeCompass::getCrownColorEnd() const
{
    return this->crownColorEnd;
}

void GaugeCompass::setCrownColorEnd(const QColor &crownColorEnd)
{
    if (this->crownColorEnd != crownColorEnd) {
        this->crownColorEnd = crownColorEnd;
        this->update();
    }
}

QColor GaugeCompass::getBgColorStart() const
{
    return this->bgColorStart;
}

void GaugeCompass::setBgColorStart(const QColor &bgColorStart)
{
    if (this->bgColorStart != bgColorStart) {
        this->bgColorStart = bgColorStart;
        this->update();
    }
}

QColor GaugeCompass::getBgColorEnd() const
{
    return this->bgColorEnd;
}

void GaugeCompass::setBgColorEnd(const QColor &bgColorEnd)
{
    if (this->bgColorEnd != bgColorEnd) {
        this->bgColorEnd = bgColorEnd;
        this->update();
    }
}

QColor GaugeCompass::getDarkColor() const
{
    return this->darkColor;
}

void GaugeCompass::setDarkColor(const QColor &darkColor)
{
    if (this->darkColor != darkColor) {
        this->darkColor = darkColor;
        this->update();
    }
}

QColor GaugeCompass::getLightColor() const
{
    return this->lightColor;
}

void GaugeCompass::setLightColor(const QColor &lightColor)
{
    if (this->lightColor != lightColor) {
        this->lightColor = lightColor;
        this->update();
    }
}

QColor GaugeCompass::getForeground() const
{
    return this->foreground;
}

void GaugeCompass::setForeground(const QColor &foreground)
{
    if (this->foreground != foreground) {
        this->foreground = foreground;
        this->update();
    }
}

QColor GaugeCompass::getTextColor() const
{
    return this->textColor;
}

void GaugeCompass::setTextColor(const QColor &textColor)
{
    if (this->textColor != textColor) {
        this->textColor = textColor;
        this->update();
    }
}

QColor GaugeCompass::getNorthPointerColor() const
{
    return this->northPointerColor;
}

void GaugeCompass::setNorthPointerColor(const QColor &northPointerColor)
{
    if (this->northPointerColor != northPointerColor) {
        this->northPointerColor = northPointerColor;
        this->update();
    }
}

QColor GaugeCompass::getSouthPointerColor() const
{
    return this->southPointerColor;
}

void GaugeCompass::setSouthPointerColor(const QColor &southPointerColor)
{
    if (this->southPointerColor != southPointerColor) {
        this->southPointerColor = southPointerColor;
        this->update();
    }
}

QColor GaugeCompass::getCenterColorStart() const
{
    return this->centerColorStart;
}

void GaugeCompass::setCenterColorStart(const QColor &centerColorStart)
{
    if (this->centerColorStart != centerColorStart) {
        this->centerColorStart = centerColorStart;
        this->update();
    }
}

QColor GaugeCompass::getCenterColorEnd() const
{
    return this->centerColorEnd;
}

void GaugeCompass::setCenterColorEnd(const QColor &centerColorEnd)
{
    if (this->centerColorEnd != centerColorEnd) {
        this->centerColorEnd = centerColorEnd;
        this->update();
    }
}
