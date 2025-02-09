#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class QPushButton;
class ImageVisualizer;
class QGroupBox;
class QGridLayout;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected slots:
    void findPath();
    void removeRow(QGridLayout *layout);
    void addRow();

private:
    ImageVisualizer* m_imageVisualizer;
    QGroupBox* m_dataGroupBox;
    bool m_isImageLoaded;
    size_t m_imageHeight;
    size_t m_imageWidth;
};
#endif // MAINWINDOW_H
