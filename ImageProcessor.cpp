#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>

#include "ImageProcessor.h"

ImageVisualizer::ImageVisualizer(QWidget *parent)
    : QWidget(parent)
{
    imageLabel = new QLabel();
    imageLabel->setAlignment(Qt::AlignCenter);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(imageLabel);
    setLayout(layout);

    setWindowTitle("Image Viewer");
    setGeometry(300, 300, 300, 200);
}

std::pair<size_t, size_t> ImageVisualizer::displayImage(const QString& fileName)
{
    QPixmap pixmap(fileName);
    imageLabel->setPixmap(pixmap);

    return {pixmap.height(), pixmap.width()};
}
