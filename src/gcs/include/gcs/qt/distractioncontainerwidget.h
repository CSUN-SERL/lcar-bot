#ifndef CONTAINERWIDGET_H
#define CONTAINERWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QList>
#include <QQueue>
#include <QMediaPlayer>

#include <gcs/qt/distractionquestionwidget.h>

namespace Ui {
class DistractionContainerWidget;
}

class DistractionContainerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DistractionContainerWidget(QWidget *parent = 0);
    ~DistractionContainerWidget();

    void AddPoint(DistractionQuestionWidget* question);
    void Reset();
    void Start();

    int GetAnsweredAmount();

private:
    Ui::DistractionContainerWidget *ui;

    QWidget* _overlay;
    QTimer* _timer;
    QMediaPlayer* _player;

    int _answeredAmount;

    QList<DistractionQuestionWidget*> _availableQuestions;
    QList<DistractionQuestionWidget*> _usedQuestions;
    QQueue<DistractionQuestionWidget*> _queueQuestions;


    void focusInEvent(QFocusEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;
    void enterEvent(QEvent *event) override;
    void leaveEvent(QEvent *event) override;

    void resizeEvent(QResizeEvent *event) override;

    void timeout();
    void initQuestions();
    void remove(DistractionQuestionWidget* question);
};

#endif // DISTRACTIONCONTAINERWIDGET_H
