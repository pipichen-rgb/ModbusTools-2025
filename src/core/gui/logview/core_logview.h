/*
    Modbus Tools

    Created: 2023
    Author: Serhii Marchuk, https://github.com/serhmarch

    Copyright (C) 2023  Serhii Marchuk

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef CORE_LOGVIEW_H
#define CORE_LOGVIEW_H

#define MBLOGVIEW_MAXSIZE_MIN 1024

#include <QWidget>
#include <mbcore.h>

class QTableView;
class QTextEdit;
class QToolBar;
class QColor;
class mbCore;
//class mbCoreLogViewModel;

class mbCoreLogView : public QWidget
{
    Q_OBJECT

public:
    struct MB_EXPORT Strings
    {
        const QString prefix;
        const QString maxSize;
        const QString font;
        const QString colors;

        Strings();
        static const Strings &instance();
    };

    struct MB_EXPORT Defaults
    {
        int maxSize;
        const QString font;
        const mb::IntColorMap colors;

        Defaults();
        static const Defaults &instance();
    };

public:
    explicit mbCoreLogView(QWidget *parent = nullptr);

public:
    inline int maxSize() const { return m_maxSize; }
    void setMaxSize(int sz);

    QString fontString() const;
    void setFontString(const QString &font);

    QVariant colorMap() const;
    void setColorMap(const QVariant &v);

    MBSETTINGS cachedSettings() const;
    void setCachedSettings(const MBSETTINGS &settings);

public:
    void logMessage(mb::LogFlag flag, const QString &source, const QString &text);
    QColor logColor(mb::LogFlag flag) const;

public Q_SLOTS:
    void clear();
    void exportLog();

Q_SIGNALS:

protected:

protected:
    mbCore *m_core;
    QToolBar *m_toolBar;
    QTextEdit *m_view;
    int m_maxSize;
    int m_offset;
    mb::IntColorMap m_colorMap;
    //QTableView *m_view;
    //mbCoreLogViewModel *m_model;
};

#endif // CORE_LOGVIEW_H
