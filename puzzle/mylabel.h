#ifndef MYLABEL
#define MYLABEL
#include "QLabel"
#include "QWidget"

class MyLabel : public QLabel
{
public:
    MyLabel(int x,int r,int c, QWidget* par) : QLabel(par)
    {
        value = x;
        if(x==-1)
            this->setText("");
        else
            this->setText(QString::number(x));
        setRC(r,c);
        this->setStyleSheet("color:rgb(85, 170, 255);background-color: rgb(255, 255, 255);border-width: 1px;border-style: solid;border-color: rgb(0, 0, 0);");
        this->setFont(QFont("Times",30));

        this->setAlignment(Qt::AlignCenter);
        this->setGeometry(QRect(c*len, r*len, 100, 100));
    }

    void setRC(int r, int c)
    {
        this->row=r;
        this->col=c;
        //this->setGeometry(QRect(col*len, row*len, 100, 100));
    }

    void changeColor(int x)
    {
        if(x==0)
            this->setStyleSheet("color:rgb(85, 170, 255);background-color: rgb(255, 255, 255);border-width: 1px;border-style: solid;border-color: rgb(0, 0, 0);");
        else
            this->setStyleSheet("color:rgb(85, 170, 255);background-color: rgb(255,255,0);border-width: 1px;border-style: solid;border-color: rgb(0, 0, 0);");
    }

public:
    int value;

private:
    const int len = 100;
    int row;
    int col;
};

class BigLabel : public QLabel
{

public:
    BigLabel(QWidget* par) : QLabel(par)
    {
        this->setStyleSheet("background-color: rgb(255, 255, 255);");
        this->setGeometry(QRect(20,20,N*len, N*len));
        this->lower();
    }
private:
    const int N=5;
    const int len = 100;
};

#endif // MYLABEL

