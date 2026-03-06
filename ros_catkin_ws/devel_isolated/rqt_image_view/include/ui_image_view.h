/********************************************************************************
** Form generated from reading UI file 'image_view.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGE_VIEW_H
#define UI_IMAGE_VIEW_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "rqt_image_view/ratio_layouted_frame.h"

QT_BEGIN_NAMESPACE

class Ui_ImageViewWidget
{
public:
    QVBoxLayout *verticalLayout;
    QWidget *toolbar_widget;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QComboBox *topics_combo_box;
    QPushButton *refresh_topics_push_button;
    QPushButton *zoom_1_push_button;
    QSpinBox *num_gridlines_spin_box;
    QCheckBox *dynamic_range_check_box;
    QDoubleSpinBox *max_range_double_spin_box;
    QPushButton *save_as_image_push_button;
    QSpacerItem *horizontalSpacer_3;
    QHBoxLayout *horizontalLayout;
    QCheckBox *publish_click_location_check_box;
    QLineEdit *publish_click_location_topic_line_edit;
    QCheckBox *smooth_image_check_box;
    QPushButton *rotate_left_push_button;
    QLabel *rotate_label;
    QPushButton *rotate_right_push_button;
    QComboBox *color_scheme_combo_box;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *image_layout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayout_4;
    rqt_image_view::RatioLayoutedFrame *image_frame;

    void setupUi(QWidget *ImageViewWidget)
    {
        if (ImageViewWidget->objectName().isEmpty())
            ImageViewWidget->setObjectName(QString::fromUtf8("ImageViewWidget"));
        ImageViewWidget->setEnabled(true);
        ImageViewWidget->resize(425, 300);
        verticalLayout = new QVBoxLayout(ImageViewWidget);
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        toolbar_widget = new QWidget(ImageViewWidget);
        toolbar_widget->setObjectName(QString::fromUtf8("toolbar_widget"));
        verticalLayout_2 = new QVBoxLayout(toolbar_widget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        topics_combo_box = new QComboBox(toolbar_widget);
        topics_combo_box->setObjectName(QString::fromUtf8("topics_combo_box"));
        topics_combo_box->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        horizontalLayout_3->addWidget(topics_combo_box);

        refresh_topics_push_button = new QPushButton(toolbar_widget);
        refresh_topics_push_button->setObjectName(QString::fromUtf8("refresh_topics_push_button"));
        QIcon icon;
        QString iconThemeName = QString::fromUtf8("view-refresh");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon = QIcon::fromTheme(iconThemeName);
        } else {
            icon.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        }
        refresh_topics_push_button->setIcon(icon);

        horizontalLayout_3->addWidget(refresh_topics_push_button);

        zoom_1_push_button = new QPushButton(toolbar_widget);
        zoom_1_push_button->setObjectName(QString::fromUtf8("zoom_1_push_button"));
        zoom_1_push_button->setEnabled(false);
        QIcon icon1;
        iconThemeName = QString::fromUtf8("zoom-original");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon1 = QIcon::fromTheme(iconThemeName);
        } else {
            icon1.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        }
        zoom_1_push_button->setIcon(icon1);
        zoom_1_push_button->setCheckable(true);

        horizontalLayout_3->addWidget(zoom_1_push_button);

        num_gridlines_spin_box = new QSpinBox(toolbar_widget);
        num_gridlines_spin_box->setObjectName(QString::fromUtf8("num_gridlines_spin_box"));
        num_gridlines_spin_box->setMinimum(0.000000000000000);
        num_gridlines_spin_box->setMaximum(20.000000000000000);
        num_gridlines_spin_box->setValue(0.000000000000000);

        horizontalLayout_3->addWidget(num_gridlines_spin_box);

        dynamic_range_check_box = new QCheckBox(toolbar_widget);
        dynamic_range_check_box->setObjectName(QString::fromUtf8("dynamic_range_check_box"));

        horizontalLayout_3->addWidget(dynamic_range_check_box);

        max_range_double_spin_box = new QDoubleSpinBox(toolbar_widget);
        max_range_double_spin_box->setObjectName(QString::fromUtf8("max_range_double_spin_box"));
        max_range_double_spin_box->setMinimum(0.010000000000000);
        max_range_double_spin_box->setMaximum(100.000000000000000);
        max_range_double_spin_box->setValue(10.000000000000000);

        horizontalLayout_3->addWidget(max_range_double_spin_box);

        save_as_image_push_button = new QPushButton(toolbar_widget);
        save_as_image_push_button->setObjectName(QString::fromUtf8("save_as_image_push_button"));
        QIcon icon2;
        iconThemeName = QString::fromUtf8("image-x-generic");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon2 = QIcon::fromTheme(iconThemeName);
        } else {
            icon2.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        }
        save_as_image_push_button->setIcon(icon2);
        save_as_image_push_button->setIconSize(QSize(16, 16));

        horizontalLayout_3->addWidget(save_as_image_push_button);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        publish_click_location_check_box = new QCheckBox(toolbar_widget);
        publish_click_location_check_box->setObjectName(QString::fromUtf8("publish_click_location_check_box"));

        horizontalLayout->addWidget(publish_click_location_check_box);

        publish_click_location_topic_line_edit = new QLineEdit(toolbar_widget);
        publish_click_location_topic_line_edit->setObjectName(QString::fromUtf8("publish_click_location_topic_line_edit"));

        horizontalLayout->addWidget(publish_click_location_topic_line_edit);

        smooth_image_check_box = new QCheckBox(toolbar_widget);
        smooth_image_check_box->setObjectName(QString::fromUtf8("smooth_image_check_box"));

        horizontalLayout->addWidget(smooth_image_check_box);

        rotate_left_push_button = new QPushButton(toolbar_widget);
        rotate_left_push_button->setObjectName(QString::fromUtf8("rotate_left_push_button"));
        QIcon icon3;
        iconThemeName = QString::fromUtf8("object-rotate-left");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon3 = QIcon::fromTheme(iconThemeName);
        } else {
            icon3.addFile(QString::fromUtf8(""), QSize(), QIcon::Normal, QIcon::Off);
        }
        rotate_left_push_button->setIcon(icon3);

        horizontalLayout->addWidget(rotate_left_push_button);

        rotate_label = new QLabel(toolbar_widget);
        rotate_label->setObjectName(QString::fromUtf8("rotate_label"));
        rotate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout->addWidget(rotate_label);

        rotate_right_push_button = new QPushButton(toolbar_widget);
        rotate_right_push_button->setObjectName(QString::fromUtf8("rotate_right_push_button"));
        QIcon icon4;
        iconThemeName = QString::fromUtf8("object-rotate-right");
        if (QIcon::hasThemeIcon(iconThemeName)) {
            icon4 = QIcon::fromTheme(iconThemeName);
        } else {
            icon4.addFile(QString::fromUtf8(""), QSize(), QIcon::Normal, QIcon::Off);
        }
        rotate_right_push_button->setIcon(icon4);

        horizontalLayout->addWidget(rotate_right_push_button);

        color_scheme_combo_box = new QComboBox(toolbar_widget);
        color_scheme_combo_box->setObjectName(QString::fromUtf8("color_scheme_combo_box"));
        color_scheme_combo_box->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        horizontalLayout->addWidget(color_scheme_combo_box);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout_2->addLayout(horizontalLayout);


        verticalLayout->addWidget(toolbar_widget);

        image_layout = new QHBoxLayout();
        image_layout->setSpacing(0);
        image_layout->setObjectName(QString::fromUtf8("image_layout"));
        scrollArea = new QScrollArea(ImageViewWidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 767, 650));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollAreaWidgetContents->sizePolicy().hasHeightForWidth());
        scrollAreaWidgetContents->setSizePolicy(sizePolicy);
        horizontalLayout_4 = new QHBoxLayout(scrollAreaWidgetContents);
        horizontalLayout_4->setContentsMargins(1, 1, 1, 1);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        image_frame = new rqt_image_view::RatioLayoutedFrame(scrollAreaWidgetContents);
        image_frame->setObjectName(QString::fromUtf8("image_frame"));
        sizePolicy.setHeightForWidth(image_frame->sizePolicy().hasHeightForWidth());
        image_frame->setSizePolicy(sizePolicy);
        image_frame->setMinimumSize(QSize(80, 60));
        image_frame->setContextMenuPolicy(Qt::ActionsContextMenu);
        image_frame->setFrameShape(QFrame::NoFrame);
        image_frame->setLineWidth(1);

        horizontalLayout_4->addWidget(image_frame);

        scrollArea->setWidget(scrollAreaWidgetContents);

        image_layout->addWidget(scrollArea);

        image_layout->setStretch(0, 1);

        verticalLayout->addLayout(image_layout);

        verticalLayout->setStretch(1, 1);

        retranslateUi(ImageViewWidget);

        QMetaObject::connectSlotsByName(ImageViewWidget);
    } // setupUi

    void retranslateUi(QWidget *ImageViewWidget)
    {
        ImageViewWidget->setWindowTitle(QCoreApplication::translate("ImageViewWidget", "Image View", nullptr));
#if QT_CONFIG(tooltip)
        refresh_topics_push_button->setToolTip(QCoreApplication::translate("ImageViewWidget", "Refresh topics", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        zoom_1_push_button->setToolTip(QCoreApplication::translate("ImageViewWidget", "Original zoom", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        num_gridlines_spin_box->setToolTip(QCoreApplication::translate("ImageViewWidget", "# of gridlines to overlay", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        dynamic_range_check_box->setToolTip(QCoreApplication::translate("ImageViewWidget", "Dynamic depth range", nullptr));
#endif // QT_CONFIG(tooltip)
        dynamic_range_check_box->setText(QString());
#if QT_CONFIG(tooltip)
        max_range_double_spin_box->setToolTip(QCoreApplication::translate("ImageViewWidget", "Max depth", nullptr));
#endif // QT_CONFIG(tooltip)
        max_range_double_spin_box->setSuffix(QCoreApplication::translate("ImageViewWidget", "m", nullptr));
#if QT_CONFIG(tooltip)
        save_as_image_push_button->setToolTip(QCoreApplication::translate("ImageViewWidget", "Save as image", nullptr));
#endif // QT_CONFIG(tooltip)
        publish_click_location_check_box->setText(QString());
#if QT_CONFIG(tooltip)
        publish_click_location_topic_line_edit->setToolTip(QCoreApplication::translate("ImageViewWidget", "Click location topic (leave empty for auto-naming)", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        smooth_image_check_box->setToolTip(QCoreApplication::translate("ImageViewWidget", "Dynamic depth range", nullptr));
#endif // QT_CONFIG(tooltip)
        smooth_image_check_box->setText(QCoreApplication::translate("ImageViewWidget", "Smooth scaling", nullptr));
        rotate_left_push_button->setText(QString());
        rotate_label->setText(QCoreApplication::translate("ImageViewWidget", "0\302\260", nullptr));
        rotate_right_push_button->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ImageViewWidget: public Ui_ImageViewWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGE_VIEW_H
