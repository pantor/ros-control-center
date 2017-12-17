import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { DashboardComponent } from './dashboard/dashboard.component';
import { SettingsComponent } from './settings/settings.component';
import { ParameterComponent } from './parameter/parameter.component';

import { TopicComponent } from './topic/topic.component';
import { TopicDefaultComponent } from './topic/default/default.component';
import { TopicStdMsgsNumberComponent } from './topic/std_msgs/Number/Number.component';
import { TopicSensorMsgsImuComponent } from './topic/sensor_msgs/Imu/Imu.component';

import { ServiceComponent } from './service/service.component';
import { ServiceDefaultComponent } from './service/default/default.component';
import { ServiceStdSrvsEmptyComponent } from './service/std_srvs/Empty/Empty.component';
// import { ServiceStdSrvsSetBoolComponent } from './service/std_srvs/SetBool/SetBool.component';
import { ServiceStdSrvsTriggerComponent } from './service/std_srvs/Trigger/Trigger.component';


@NgModule({
  declarations: [
    AppComponent,
    DashboardComponent,
    SettingsComponent,
    ParameterComponent,
    TopicComponent,
    TopicDefaultComponent,
    TopicStdMsgsNumberComponent,
    TopicSensorMsgsImuComponent,
    ServiceComponent,
    ServiceDefaultComponent,
    ServiceStdSrvsEmptyComponent,
    ServiceStdSrvsTriggerComponent,
  ],
  entryComponents: [],
  imports: [
    BrowserModule,
    FormsModule,
    AppRoutingModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
