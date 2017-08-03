#include <QUrl>

#include "ui_distractioncontainerwidget.h"

#include <gcs/qt/distractioncontainerwidget.h>
#include <gcs/qt/distractionquestionwidget.h>

DistractionContainerWidget::DistractionContainerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DistractionContainerWidget),
    _timer(new QTimer(this)),
    _overlay(new QWidget(this)),
    _answeredAmount(0),
    _player(new QMediaPlayer())
{
    ui->setupUi(this);
    initQuestions();
    QObject::connect(_timer, &QTimer::timeout, this, &DistractionContainerWidget::timeout);

    ui->scrollArea->stackUnder(_overlay);

    _overlay->setStyleSheet("background-color: rgba(0,0,0,125)");
    _overlay->hide();

    //setFocusPolicy(Qt::ClickFocus);
    QMediaContent audio(QUrl("qrc:/Resources/Pling.mp3"));
    _player->setMedia(audio);
    _player->setVolume(100);
}

DistractionContainerWidget::~DistractionContainerWidget()
{
    delete ui;
}

void DistractionContainerWidget::AddPoint(DistractionQuestionWidget *question)
{
    _answeredAmount++;
    remove(question);
    if(_usedQuestions.length() == 0)
    {
        timeout();
    }
}

void DistractionContainerWidget::remove(DistractionQuestionWidget *question)
{
    int index = _usedQuestions.indexOf(question);
    _usedQuestions.removeAt(index);
    _queueQuestions.enqueue(question);
    if(_queueQuestions.length() > _availableQuestions.length())
    {
        _availableQuestions.append(_queueQuestions.dequeue());
    }

    question->hide();
    ui->_contents->removeWidget(question);
}

void DistractionContainerWidget::Reset()
{
    _timer->stop();
    _answeredAmount = 0;
    while(_usedQuestions.length() > 0)
    {
        remove(_usedQuestions[0]);
    }
}

void DistractionContainerWidget::Start()
{
    timeout();
}

int DistractionContainerWidget::GetAnsweredAmount()
{
    return _answeredAmount;
}

void DistractionContainerWidget::timeout()
{
    //10seconds - 20seconds
    _timer->start((qrand() % 10000) + 10000);
    if(_availableQuestions.length() > 0)
    {
        _player->stop();
        _player->play();


        int index = qrand()%_availableQuestions.length();
        _availableQuestions[index]->show();
        ui->_contents->addWidget(_availableQuestions[index]);
        _usedQuestions.append(_availableQuestions[index]);
        _availableQuestions.removeAt(index);
    }
}

void DistractionContainerWidget::focusInEvent(QFocusEvent *event)
{
    QWidget::focusInEvent(event);
    _overlay->hide();
}

void DistractionContainerWidget::focusOutEvent(QFocusEvent *event)
{
    QWidget::focusOutEvent(event);
    _overlay->show();
}

void DistractionContainerWidget::enterEvent(QEvent *event)
{
    _overlay->hide();
}

void DistractionContainerWidget::leaveEvent(QEvent *event)
{
    _overlay->show();
}

void DistractionContainerWidget::resizeEvent(QResizeEvent *event)
{
    _overlay->resize(size());
}

void DistractionContainerWidget::initQuestions()
{
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT1.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT1.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT1.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT1.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many steps are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT1.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image white?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT2.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many atennae are there on the building in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT2.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT2.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT2.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many doors are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT2.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many stories is the building in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT3.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT3.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT3.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT3.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many windows are shown on the black building in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT3.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the third building from the left yellow in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT4.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT4.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT4.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT4.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many garage doors does this building have?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT4.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many stories is the building in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT5.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT5.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT5.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT5.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken on a sunny day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT5.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many red cars are shown in this image?", DistractionQuestionWidget::Input));\


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT6.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image blue?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT6.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT6.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT6.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT6.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many windows does the building in this image have?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT7.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT7.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT7.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT7.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the roof intact in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT7.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many stories is the building in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT8.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image blue?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT8.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT8.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT8.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT8.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image white?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT9.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT9.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT9.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT9.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT9.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many stories is the building in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT10.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT10.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT10.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT10.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT10.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image green?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT11.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT11.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT11.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT11.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT11.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flower patches are shown in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT12.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT12.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT12.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT12.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are all of the flags in this image the same?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT12.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many lights does each lamppost have in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT13.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT13.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT13.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT13.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are all of the flags in this image the ame?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT13.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT14.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT14.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many white cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT14.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT14.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is it raining in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT14.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many stories is the building on the left of this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT15.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the building in this image beige?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT15.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many emergency medical vehicles are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT15.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT15.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT15.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT16.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are the buildings in this image blue?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT16.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the fire hydrant in this image more than one color?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT16.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hygrants are in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT16.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT16.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the top of the fire hydrant blue in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT17.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT17.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT17.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT17.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many roads are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT17.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many white umbrellas are shown in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT18.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many blue buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT18.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many white buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT18.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are parked in the bottom-right park lot in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT18.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many yellow cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT18.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT19.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT19.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many raised driveways are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT19.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT19.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many parking lots are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT19.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was it raining when this image was taken?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT20.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many red buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT20.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are the four largest buildings in the image the same size?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT20.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many white buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT20.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many street intersections are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT20.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT21.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT21.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are the two white buildings in the image the same size?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT21.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are there more than 20 cars in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT21.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT21.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Are there any planes or other flying vehicles in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT22.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many airplanes are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT22.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many city blocks are shown in this picture?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT22.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is there grass shown in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT22.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many intersections are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT22.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT23.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT23.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many black cars are parked in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT23.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many red cars are parked in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT23.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT23.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the location shown in this image a school?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT24.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many flags are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT24.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many floors is the building shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT24.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT24.jpeg").scaled(500,500,Qt::KeepAspectRatio), "Is the car in the bottom-right of this image white?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT24.jpeg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT25.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars appear in the closest parking row in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT25.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT25.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT25.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT25.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT26.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT26.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is the closest car in this image blue?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT26.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is the closest car in this image white?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT26.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT26.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT27.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT27.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT27.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are clouds visible in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT27.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT27.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are both the buildings shown in this image the same color?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT28.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many blue cars are visible in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT28.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT28.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT28.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT28.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is water shown in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT29.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT29.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars appear in the closest parking row in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT29.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT29.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT29.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT30.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT30.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT30.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are clouds visible in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT30.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT30.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many palm trees are visible in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT31.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT31.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT31.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT31.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many green cars appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT31.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many red cars appear in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT32.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT32.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT32.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many green cars appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT32.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT32.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many red cars appear in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT33.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many building are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT33.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT33.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are clouds visible in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT33.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT33.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many police cars are visible in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT34.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT34.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT34.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT34.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many police cars are visible in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT34.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is water shown in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT35.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many blue cars are visible in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT35.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT35.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT35.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT35.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many stories high are the buildings in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT36.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT36.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT36.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT36.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many palm trees are visible in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT36.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many police cars are visible in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT37.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many blue cars are visible in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT37.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many buildings are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT37.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT37.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT37.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many palm trees are visible in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT38.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT38.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT38.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT38.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is water shown in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT38.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many white cars appear in this image?", DistractionQuestionWidget::Input));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT39.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT39.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are clouds visible in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT39.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the day?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT39.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many fire hydrants appear in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT39.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is the bus yellow in this image?", DistractionQuestionWidget::YesNo));


    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT40.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many cars are shown in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT40.jpg").scaled(500,500,Qt::KeepAspectRatio), "Are clouds visible in this image?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT40.jpg").scaled(500,500,Qt::KeepAspectRatio), "Was this image taken during the night?", DistractionQuestionWidget::YesNo));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT40.jpg").scaled(500,500,Qt::KeepAspectRatio), "How many stories high are the buildings in this image?", DistractionQuestionWidget::Input));
    _availableQuestions.append(new DistractionQuestionWidget(this, QPixmap(":/Resources/CVT40.jpg").scaled(500,500,Qt::KeepAspectRatio), "Is water shown in this image?", DistractionQuestionWidget::YesNo));


    for(int i = 0; i < _availableQuestions.length(); i++)
    {
        ui->_contents->addWidget(_availableQuestions[i]);
        _availableQuestions[i]->hide();
    }
}
