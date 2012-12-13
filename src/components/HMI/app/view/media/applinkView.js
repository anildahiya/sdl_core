/**
 * @name MFT.applinkView
 * 
 * @desc Applink Media application module visual representation
 * 
 * @category	View
 * @filesource	app/view/media/applinkView.js
 * @version		2.0
 *
 * @author		Melnik Andriy
 */
MFT.applinkView = Em.ContainerView.create(MFT.LoadableView,{
	
	/** View Id */	
	elementId: 'applink_view_container',

	/** View Components*/
	childViews: [
		'controlls',
		'info',
		'innerMenu'
	],
	
	innerMenu: MFT.MenuList.extend({
					
		content: Em.ContainerView.extend({
			
			classNames: ['content'],
			
			attributeBindings: ['parentView.contentPositon:style'],
			
			childViews: [
				'softButtons'
			],

		    AddSoftButton: function( params ){

		    	for(var i=0; i < params.length; i++){
			        this.get('softButtons.list.childViews').pushObject(
			            MFT.Button.create({
			                actionDown:		function(){
								this._super();
								FFW.Buttons.buttonEventCustom( "CUSTOM_BUTTON", "BUTTONDOWN", this.softButtonID);
								var self = this;
								this.time = 0;
								setTimeout(function(){ self.time ++; }, 1000);
							},
							actionUp:		function(){
								this._super();
								FFW.Buttons.buttonEventCustom( "CUSTOM_BUTTON", "BUTTONUP", this.softButtonID);
								if(this.time > 0){
									FFW.Buttons.buttonPressedCustom( "CUSTOM_BUTTON", "LONG", this.softButtonID);
								}else{
									FFW.Buttons.buttonPressedCustom( "CUSTOM_BUTTON", "SHORT", this.softButtonID);
								}
								this.time = 0;
							},
			                softButtonID:           params[i].softButtonID,
			                appId:                  appId,
			                icon:                   params[i].image,
			                text:                   params[i].text,
			                classNames:             'list-item',
			                templateName:           params[i].image ? 'rightIcon' : 'text'
			            })
			        );
			    }

		    },

			softButtons: MFT.Button.extend({
				text: 'Options',
				
				templateName: 'arrow',
				
				action:		'turnOnApplinkOptions',
				target:		'MFT.ApplinkMediaController'
			})
		})
	}),

	/** Calls Applink SystemContext switcher when turn On/Of Applink application */
	onTurnOnApplinkApp: function(systemContextValue){
		if(MFT.States.media.applink.active){
            MFT.ApplinkMediaController.onSystemContextSwitcher(MFT.ApplinkMediaController.eSystemContext.application);
		}else{
            MFT.ApplinkMediaController.onSystemContextSwitcher(MFT.ApplinkMediaController.eSystemContext.main);
		}
	}.observes('MFT.States.media.applink.active'),
	
	info: Em.View.extend({
		elementId: 'app_view_info',
		
		template: Em.Handlebars.compile(
			'{{#with view}}'+
			'<div class="track-info">'+
				'<div class="device">{{MFT.ApplinkMediaModel.showInfo.deviceName}}</div>'+
		            	'<div class="divider_o"></div>'+
				'<div class="title">{{MFT.ApplinkMediaModel.showInfo.field1}}</div>'+
				'<div class="album">{{MFT.ApplinkMediaModel.showInfo.field2}}</div>'+
				'<div class="artist">{{MFT.ApplinkMediaModel.showInfo.field3}}</div>'+
				'<div class="time">{{MFT.ApplinkMediaModel.showInfo.mediaClock}}</div>'+
//				'<div id="cd_logo" {{bindAttr class="MFT.CDModel.active:visible_display MFT.AppModel.active:visible_display MFT.helpMode:helpmode"}}></div>'+
//				'<div id="usb_logo" {{bindAttr class="MFT.USBModel.active:visible_display MFT.helpMode:helpmode"}}></div>'+
			'</div>'+
			'{{/with}}'
		)
	}),
	
	controlls: Em.ContainerView.extend({
		
		classNameBindings: ['MFT.helpMode:hidden'],
		
		elementId: 'app_view_controlls',
		/** View components*/
		childViews: [
			'PrevTrackButton',
			'PlayButton',
			'NextTrackButton'
		],
		
		classNames: 'player_controlls',
		
		PrevTrackButton: MFT.Button.extend({
			elementId: 			'app_view_controlls_prev_track_button',
			classNames:			['bc-item-big', 'prevcd'],
			classNames:			['bc-item-big', 'prevcd'],
			target:				'MFT.MediaController',
			action:				'prevTrack',
			icon:				'images/media/ico_prew.png',
		}),
		PlayButton: MFT.Button.extend({
			elementId: 			'app_view_controlls_play_button',
			classNames:			['bc-item-big', 'playcd'],
			target:				'MFT.MediaController',
			action:				'playTrack',
			/** Define button template */
			template: Ember.Handlebars.compile(
				'<img class="playIcon hideicon"{{bindAttr class="MFT.MediaController.currentSelectedPlayer.isPlaying:visible"}} src="images/media/ico_pause.png" />'+
				'<img class="playIcon showicon"{{bindAttr class="MFT.MediaController.currentSelectedPlayer.isPlaying:not-visible"}} src="images/media/ico-play.png" />'
			)
		}),
		NextTrackButton: MFT.Button.extend({
			elementId: 			'app_view_controlls_next_track_button',
			classNames:			['bc-item-big', 'nextcd'],
			target:				'MFT.MediaController',
			action:				'nextTrack',
			icon:				  'images/media/ico_next.png',
		})
	})
});