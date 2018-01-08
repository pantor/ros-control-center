import { Component, OnInit } from '@angular/core';

import { Setting } from './setting';


@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.css']
})
export class SettingsComponent implements OnInit {
  settings: Setting[];
  index: number;
  private storageSettingsName = 'roscc2-settings';
  private storageIndexName = 'roscc2-index';

  ngOnInit() {
    this.settings = JSON.parse(localStorage.getItem(this.storageSettingsName)) || [ Setting.getDefault() ];
    this.index = JSON.parse(localStorage.getItem(this.storageIndexName)) || 0;
  }

  save(): void {
    localStorage.setItem(this.storageSettingsName, JSON.stringify(this.settings));
    localStorage.setItem(this.storageIndexName, JSON.stringify(this.index));
  }

  add(): void {
    this.settings.push( Setting.getDefault() ); // Clone object
    this.index = this.settings.length - 1;
    this.save();
  }

  remove(): void {
    this.settings.splice(this.index, 1);
    this.index = 0;

    if (!this.settings.length) {
      this.add();
    }
    this.save();
  }
}
