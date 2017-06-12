import { Component, OnInit } from '@angular/core';

import { Setting } from '../setting';


@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.css']
})
export class SettingsComponent implements OnInit {
  settings: Setting[];
  index: number;

  constructor() { }

  ngOnInit() {
    this.settings = JSON.parse(localStorage.getItem('roscc2-settings')) || [ Setting.getDefault() ];
    this.index = JSON.parse(localStorage.getItem('roscc2-index')) || 0;
  }

  save(): void {
    localStorage.setItem('roscc2-settings', JSON.stringify(this.settings));
    localStorage.setItem('roscc2-index', JSON.stringify(this.index));
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
