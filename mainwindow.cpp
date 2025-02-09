#include <functional>
#include <iostream>

#include <QSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QButtonGroup>
#include <QLineEdit>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>

#include "PathProcessor.h"
#include "ImageProcessor.h"
#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_isImageLoaded = false;
    m_imageVisualizer = new ImageVisualizer();
    this->setCentralWidget(m_imageVisualizer);

    QDockWidget* rightDockWidget = new QDockWidget(this);
    rightDockWidget->setAllowedAreas(Qt::RightDockWidgetArea);
    this->addDockWidget(Qt::RightDockWidgetArea, rightDockWidget);

    QWidget* rightDockWidgetContent = new QWidget(rightDockWidget);
    QVBoxLayout* rightDockLayout = new QVBoxLayout(rightDockWidgetContent);
    rightDockWidget->setWidget(rightDockWidgetContent);

    QGroupBox* controlGroupBox = new QGroupBox("Controllers");
    QFormLayout* controlerLayout = new QFormLayout(controlGroupBox);

    QPushButton* loadImageButton = new QPushButton("Load Image", rightDockWidgetContent);
    QPushButton* loadOverrideButton = new QPushButton("Load Override", this);
    QPushButton* loadElevationButton = new QPushButton("Load Elevation", this);

    QLineEdit* imagePathEdit = new QLineEdit(this);
    QLineEdit* overridePathEdit = new QLineEdit(this);
    QLineEdit* elevationPathEdit = new QLineEdit(this);

    controlerLayout->addRow(loadImageButton, imagePathEdit);
    controlerLayout->addRow(loadOverrideButton, overridePathEdit);
    controlerLayout->addRow(loadElevationButton, elevationPathEdit);

    QPushButton* addDestButton = new QPushButton("Add New Distination", rightDockWidgetContent);
    controlerLayout->addWidget(addDestButton);

    QPushButton* findPathButton = new QPushButton("Find Path", rightDockWidgetContent);
    controlerLayout->addWidget(findPathButton);

    m_dataGroupBox    = new QGroupBox("Destinations");
    m_dataGroupBox->setLayout(new QGridLayout);
    rightDockLayout->addWidget(controlGroupBox, 0);
    rightDockLayout->addWidget(m_dataGroupBox, 1);


    auto loadFile = [this](std::function<void(const QString&)> callback, QLineEdit* lineEdit, const QString& extensions) {
        QString filePath = QFileDialog::getOpenFileName(this, "Select File", "", extensions);
        if (!filePath.isEmpty()) {
            lineEdit->setText(filePath);
            callback(filePath);
        }
    };

    auto parseFile = [=](const QString& filename){
        data_t data;
        if(m_isImageLoaded)
        {
            std::ifstream in(filename.toStdString(), std::ifstream::ate | std::ifstream::binary);
            if (!in.good())
            {
                throw std::exception();
            }
            size_t fsize = in.tellg();
            if (fsize != m_imageWidth*m_imageHeight)
            {
                throw std::exception();
            }
            data.resize(fsize);

            in.read((char*)&data[0], fsize);
        }
        else
        {
            QMessageBox::warning(this, "Warning", "Please select an image first.");
        }
        return data;
    };

    connect(addDestButton, &QPushButton::clicked, this, &MainWindow::addRow);
    connect(findPathButton, &QPushButton::clicked, this, &MainWindow::findPath);

    loadImageButton->connect(loadImageButton, &QPushButton::clicked, [=]() {
        loadFile([this](const QString& path) {
            std::tie(this->m_imageHeight, this->m_imageWidth) = m_imageVisualizer->displayImage(path);
            m_isImageLoaded = true;
        }, imagePathEdit, "Image Files (*.png *.jpg *.jpeg *.bmp)");
    });

//    loadOverrideButton->connect(loadOverrideButton, &QPushButton::clicked, [=]() {
//        loadFile([=](const QString& path) {
//            parseFile(path);
//        }, overridePathEdit, "override Files (*.txt *.bin *.data)");
//    });

//    loadElevationButton->connect(loadElevationButton, &QPushButton::clicked, [=]() {
//        loadFile([=](const QString& path) {
//            parseFile(path);
//        }, elevationPathEdit, "override Files (*.txt *.bin *.data)");
//    });
}

MainWindow::~MainWindow()
{
}

void MainWindow::findPath()
{
//    Processing::process();
}

void MainWindow::addRow() {
    if(m_isImageLoaded)
    {
        QGridLayout* dataLayout = static_cast<QGridLayout*>(m_dataGroupBox->layout());
        int row = dataLayout->rowCount();

        // Create widgets for the row
        QSpinBox* xLineEdit = new QSpinBox();
        QSpinBox* yLineEdit = new QSpinBox();
        QPushButton* removeButton = new QPushButton("-", m_dataGroupBox);

        xLineEdit->setMinimum(0);
        yLineEdit->setMinimum(0);
        xLineEdit->setMaximum(m_imageWidth);
        yLineEdit->setMaximum(m_imageHeight);

        // Connect remove button to slot
        removeButton->connect(removeButton, &QPushButton::clicked, [=](){removeRow(dataLayout);});

        // Add widgets to the layout
        dataLayout->addWidget(removeButton, row, 0);
        dataLayout->addWidget(new QLabel("x:", m_dataGroupBox), row, 1);
        dataLayout->addWidget(xLineEdit, row, 2);
        dataLayout->addWidget(new QLabel("y:", m_dataGroupBox), row, 3);
        dataLayout->addWidget(yLineEdit, row, 4);
    }
    else
    {
        QMessageBox::warning(this, "Warning", "Please select an image first.");
    }
}

void MainWindow::removeRow(QGridLayout* layout) {
    if (!layout) return;

    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QWidget* widget = item->widget()) {
            widget->deleteLater();
        }

        if (QLayout* childLayout = item->layout()) {
            removeRow(dynamic_cast<QGridLayout*>(childLayout));
        }

        delete item;
    }
    delete layout;
}

/**
 * @brief parsing the data files
 * @param pname: project location
 * @param elevation: elevations of the cells of the map
 * @param overrides: cell types of the map "river, land, ..."
 */
//void Processing::preprocess(std::string pname, data_t& elevation,
//                            data_t& overrides)
//{
//    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
//    // Address assets relative to application location
//    std::string anchor = std::string(".") + PATH_SEP;
//    auto lastpos = pname.find_last_of("/\\");
//    if (lastpos != std::string::npos)
//    {
//        anchor = pname.substr(0, lastpos) + PATH_SEP;
//    }

//    elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data",
//                         expectedFileSize);
//    overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data",
//                         expectedFileSize);
//}

