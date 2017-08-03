#include <QPixmap>
#include <QIntValidator>
#include <QKeyEvent>

#include "ui_distractionquestionwidget.h"

#include <gcs/qt/distractionquestionwidget.h>
#include <gcs/qt/distractioncontainerwidget.h>


DistractionQuestionWidget::DistractionQuestionWidget(DistractionContainerWidget* container, const QPixmap& pixmap, const QString& question, DistractionQuestionType questionType) :
    QWidget(nullptr),
    ui(new Ui::DistractionQuestionWidget),
    _container(container),
    _questionType(questionType)
{
    ui->setupUi(this);
    ui->_image->setPixmap(pixmap);
    ui->_questionLabel->setText(question);
    if(questionType == DistractionQuestionType::YesNo)
    {
        ui->_textBox->hide();
    }
    else
    {
        ui->_noButton->hide();
        ui->_yesButton->setText("Enter");
    }
    ui->_textBox->setValidator(new QIntValidator(0, INT32_MAX));
    QObject::connect(ui->_yesButton, &QPushButton::clicked, this, &DistractionQuestionWidget::yesClicked);
    QObject::connect(ui->_noButton, &QPushButton::clicked, this, &DistractionQuestionWidget::noClicked);
}

DistractionQuestionWidget::~DistractionQuestionWidget()
{
    delete ui;
}

void DistractionQuestionWidget::keyPressEvent(QKeyEvent *event)
{
    if(selected &&
        (event->key() == Qt::Key_Enter ||
        event->key() == Qt::Key_Return))
    {
        yesClicked();
    }
}

void DistractionQuestionWidget::yesClicked()
{
    if(_questionType == DistractionQuestionType::Input)
    {
        if(!ui->_textBox->text().isEmpty())
        {
            ui->_textBox->setText("");
            _container->AddPoint(this);
        }
        return;
    }
    _container->AddPoint(this);
}

void DistractionQuestionWidget::noClicked()
{
    _container->AddPoint(this);
}
