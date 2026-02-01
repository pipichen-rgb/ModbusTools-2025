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
#include "core_statisticsmanager.h"

#include <QMdiArea>
#include <QMdiSubWindow>

#include <core.h>

#include "core_portstatisticsui.h"
#include <project/core_project.h>
#include <project/core_port.h>
#include <project/core_dataview.h>

mbCoreStatisticsManager::mbCoreStatisticsManager(QObject *parent) : QObject(parent)
{
    m_project = nullptr;
    m_activeStatisticsUi = nullptr;

    mbCore *core = mbCore::globalCore();
    connect(core, &mbCore::projectChanged, this, &mbCoreStatisticsManager::setProject);
}

void mbCoreStatisticsManager::addPortStatisticsUi(mbCorePort *port)
{
    portStatisticsAdd(port);
}

void mbCoreStatisticsManager::setActiveStatisticsUi(mbCoreStatisticsUi *ui)
{
    // TODO: ASSERT m_hashStatisticsUis.contains(ui->statistics())
    if (m_activeStatisticsUi != ui)
    {
        m_activeStatisticsUi = ui;
        Q_EMIT statisticsUiActivated(ui);
    }
}

void mbCoreStatisticsManager::setProject(mbCoreProject *p)
{
    mbCoreProject *project = static_cast<mbCoreProject*>(p);
    if (m_project != project)
    {
        if (m_project)
        {
            Q_FOREACH (mbCorePort* p, m_project->portsCore())
                portStatisticsRemove(p);
            m_project->disconnect(this);
            m_activeStatisticsUi = nullptr;
        }
        m_project = project;
        if (m_project)
        {
            connect(m_project, &mbCoreProject::portRemoving, this, &mbCoreStatisticsManager::portStatisticsRemove);
        }
    }
}

void mbCoreStatisticsManager::portStatisticsAdd(mbCorePort *port)
{
    // TODO: ASSERT !m_hashPortStatisticsUis.contains(port)
    mbCorePortStatisticsUi *ui = createPortStatisticsUi(port);
    m_portStatisticsUis.append(ui);
    m_hashPortStatisticsUis.insert(port, ui);
    Q_EMIT statisticsUiAdd(ui);
}

void mbCoreStatisticsManager::portStatisticsRemove(mbCorePort *port)
{
    // TODO: ASSERT ui != nullptr && m_hashPortStatisticsUis.contains(ui->portStatistics())
    mbCorePortStatisticsUi *ui = portStatisticsUiCore(port);
    if (!ui)
        return;
    if (m_activeStatisticsUi == ui)
        m_activeStatisticsUi = nullptr;
    ui->disconnect(this);
    Q_EMIT statisticsUiRemove(ui);
    m_portStatisticsUis.removeOne(ui);
    m_hashPortStatisticsUis.remove(port);
    ui->deleteLater(); // Note: need because 'ui' can have 'QMenu'-children located in stack which is trying to delete
}
