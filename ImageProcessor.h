#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QWidget>

class QLabel;

class ImageVisualizer : public QWidget
{
    Q_OBJECT
public:
    explicit ImageVisualizer(QWidget *parent = nullptr);
    std::pair<size_t, size_t> displayImage(const QString &fileName);


private:
    QLabel* imageLabel;
};

#endif // IMAGEPROCESSOR_H
