// SPDX-License-Identifier: LGPL-2.1-or-later

#include <QDebug>
#include <QLayout>
#include <QMainWindow>
#include <QTest>

#include <App/Application.h>

#include "Gui/QuantitySpinBox.h"

// NOLINTBEGIN(readability-magic-numbers)

class testQuantitySpinBox: public QObject
{
    Q_OBJECT

public:
    testQuantitySpinBox()
        : qsb(nullptr)
        , mainWindow(nullptr)
    {
        if (App::Application::GetARGC() == 0) {
            constexpr int argc = 1;
            std::array<char*, argc> argv {"FreeCAD"};
            App::Application::Config()["ExeName"] = "FreeCAD";
            App::Application::init(argc, argv.data());
        }
        auto topLevel = qApp->topLevelWindows();

        for (const auto window : topLevel) {
            if (auto mw = qobject_cast<QMainWindow*>(window)) {
                mainWindow = mw;
                break;
            }
        }
        if (!mainWindow) {
            mainWindow = new QMainWindow();
        }
        qsb = new Gui::QuantitySpinBox;
        if (!mainWindow->layout()) {
            mainWindow->setLayout(new QVBoxLayout);
        }
        mainWindow->layout()->addWidget(qsb);
        qsb->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mainWindow->show();
    }

private Q_SLOTS:

    void init()
    {}

    void cleanup()
    {}

    void test_SimpleBaseUnit()// NOLINT
    {
        auto result = qsb->valueFromText("1mm");
        QCOMPARE(result, Base::Quantity(1, QLatin1String("mm")));
    }

    void test_UnitInNumerator()// NOLINT
    {
        auto result = qsb->valueFromText("1mm/10");
        QCOMPARE(result, Base::Quantity(0.1, QLatin1String("mm")));
    }

    void test_UnitInDenominator()// NOLINT
    {
        auto result = qsb->valueFromText("1/10mm");
        QCOMPARE(result, Base::Quantity(0.1, QLatin1String("mm")));
    }

    void test_DefaultUnitLost()// NOLINT
    {
        // Arrange
        auto foundWindow = QTest::qWaitForWindowActive(mainWindow);
        QTEST_ASSERT(foundWindow);
        qsb->activateWindow();
        qsb->clear();
        QTest::keyClicks(qsb, "1um");
        QTest::keyClick(qsb, Qt::Key_Enter);
        QTEST_ASSERT(qsb->value() == Base::Quantity(1, QLatin1String("um")));
        QTest::qWait(5000);

        // Act
        QTest::mouseClick(qsb, Qt::MouseButton::LeftButton);
        QTest::keyClicks(qsb, "3");
        QTest::keyClick(qsb, Qt::Key_Enter);
        qsb->evaluateExpression();
        QTest::qWait(5000);

        // Assert
        QCOMPARE(qsb->value(), Base::Quantity(3, QLatin1String("um")));
    }

private:
    Gui::QuantitySpinBox* qsb;
    QMainWindow* mainWindow;
};

// NOLINTEND(readability-magic-numbers)

QTEST_MAIN(testQuantitySpinBox)

#include "QuantitySpinBox.moc"
