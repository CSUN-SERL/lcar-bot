#ifndef QUESTIONWIDGET_H
#define QUESTIONWIDGET_H

#include <QWidget>

class DistractionContainerWidget;

namespace Ui {
class DistractionQuestionWidget;
}

class DistractionQuestionWidget : public QWidget
{
    Q_OBJECT

public:
    enum DistractionQuestionType{
        YesNo,
        Input
    };

    explicit DistractionQuestionWidget(DistractionContainerWidget* container, const QPixmap& pixmap, const QString& question, DistractionQuestionType questionType);
    ~DistractionQuestionWidget();

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    Ui::DistractionQuestionWidget *ui;
    DistractionContainerWidget* _container;
    DistractionQuestionType _questionType;

    bool selected;

private slots:
    void yesClicked();
    void noClicked();
};

#endif // DISTRACTIONQUESTIONWIDGET_H
