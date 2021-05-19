#include "mainwindow.h"
#define tt 100
#define tt1 50
#define tt2 90

queue<Movement> MainWindow::moveQueue;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    BigLabel* biglabel = new BigLabel(this);

    fstream fin("/Users/vamsh/Desktop/puzzle/data_normal.txt");
    for(int r=0;r<N;r++)
        for(int c=0;c<N;c++)
            fin>>destArr[r][c];

    for(int r=0;r<N;r++)
        for(int c=0;c<N;c++)
        {
            int x;
            fin>>x;
            if(x==0)
                labelArr[r][c] = NULL;
            else
            {
                labelArr[r][c] = new MyLabel(x,r,c,biglabel);
                if(labelArr[r][c]->value==destArr[r][c])
                     labelArr[r][c]->changeColor(1);
            }
        }

    int r1,c1,r2,c2;
    while(fin>>r1>>c1>>r2>>c2)
    {
        MainWindow::moveQueue.push(Movement(r1,c1,r2,c2));
    }

    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(start()));
}

void MainWindow::start()
{
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
    timer->start(tt+tt2);
}

void MainWindow::timerUpdate()
{
    if(MainWindow::moveQueue.empty())
        return;

    exchange(MainWindow::moveQueue.front().r1, MainWindow::moveQueue.front().c1,
             MainWindow::moveQueue.front().r2, MainWindow::moveQueue.front().c2);
    MainWindow::moveQueue.pop();
}

void MainWindow::exchange(int r1, int c1, int r2, int c2)
{
    QPropertyAnimation *anim1 = new QPropertyAnimation(labelArr[r1][c1],"pos");
    anim1->setDuration(tt);
    anim1->setStartValue(getXY(r1,c1));
    anim1->setEndValue(getXY(r2,c2));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(changeColor()));
    timer->setSingleShot(true);
    timer->start(tt+tt1);
    anim1->start();

    labelArr[r2][c2] = labelArr[r1][c1];
    labelArr[r1][c1] = NULL;
    rr = r2;
    cc = c2;

//    labelArr[r1][c1]->setRC(r2,c2);

}

void MainWindow::changeColor()
{
    if(labelArr[rr][cc]->value==destArr[rr][cc])
         labelArr[rr][cc]->changeColor(1);
    else
        labelArr[rr][cc]->changeColor(0);
}


QPoint MainWindow::getXY(int r, int c)
{
    int x = c*len;
    int y = r*len;
    return QPoint(x,y);
}

MainWindow::~MainWindow()
{
    delete ui;
}
