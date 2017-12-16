import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { FormsModule }   from '@angular/forms';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { DashboardComponent } from './dashboard/dashboard.component';
import { SettingsComponent } from './settings/settings.component';
import { ParameterComponent } from './parameter/parameter.component';
import { TopicComponent } from './topic/topic.component';
import { ServiceComponent } from './service/service.component';
import { DefaultComponent } from './topic/default/default.component';

@NgModule({
  declarations: [
    AppComponent,
    DashboardComponent,
    SettingsComponent,
    ParameterComponent,
    TopicComponent,
    ServiceComponent,
    DefaultComponent
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
