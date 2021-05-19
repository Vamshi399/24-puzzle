#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define nn 5
#include <QMainWindow>
#include <QPropertyAnimation>
#include "ui_mainwindow.h"
#include "mylabel.h"
#include "fstream"
#include "qtimer.h"
#include "queue"
using namespace std;
namespace Ui {
class MainWindow;
}

struct Movement
{
    int r1,c1,r2,c2;
    Movement(int r1, int c1, int r2, int c2)
    {
        this->r1=r1;this->r2=r2;
        this->c1=c1;this->c2=c2;
    }
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    QPoint getXY(int,int);
    void exchange(int, int, int, int);
    ~MainWindow();

protected slots:
    void start();
    void timerUpdate();
    void changeColor();

private:
    Ui::MainWindow *ui;
    const int N=nn;
    const int len=100;
    int rr;
    int cc;
    MyLabel* labelArr[nn][nn];
    int destArr[nn][nn];
public:
    static queue<Movement> moveQueue;
};

#endif // MAINWINDOW_H
